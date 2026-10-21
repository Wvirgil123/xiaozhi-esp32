#include "face_database.h"
#include <nvs_flash.h>
#include <esp_log.h>
#include <cstring>
#include <cstdint>
#include <cmath>

static const char* TAG = "FaceDatabase";
static const char* NVS_NAMESPACE = "face_db";

FaceDatabase& FaceDatabase::GetInstance() {
    static FaceDatabase instance;
    return instance;
}

FaceDatabase::FaceDatabase() : loaded_(false) {
    LoadFromNvs();
}

float FaceDatabase::CosineSimilarity(const float* a, const float* b, int dim) {
    float dot = 0.0f;
    float norm_a = 0.0f;
    float norm_b = 0.0f;

    for (int i = 0; i < dim; i++) {
        dot += a[i] * b[i];
        norm_a += a[i] * a[i];
        norm_b += b[i] * b[i];
    }

    float denom = sqrtf(norm_a) * sqrtf(norm_b);
    if (denom < 1e-8f) {
        return 0.0f;
    }

    return dot / denom;
}

void FaceDatabase::NormalizeEmbedding(float* embedding, int dim) {
    float norm = 0.0f;
    for (int i = 0; i < dim; i++) {
        norm += embedding[i] * embedding[i];
    }

    norm = sqrtf(norm);
    if (norm > 1e-8f) {
        for (int i = 0; i < dim; i++) {
            embedding[i] /= norm;
        }
    }
}

// ---- fp16 <-> float32 (IEEE-754 binary16) -------------------------------
// Software conversion used only at the NVS/wire boundary. faces_ stays float32.
// Endianness of the 16-bit value itself is the host's; the LE byte ordering of
// the *blob* is handled by the byte-wise pack/unpack at the call sites.

uint16_t FaceDatabase::Float32ToHalf(float f) {
    uint32_t x;
    memcpy(&x, &f, sizeof(x));

    uint32_t sign = (x >> 16) & 0x8000u;
    uint32_t mant = x & 0x007FFFFFu;
    uint32_t biased = (x >> 23) & 0xFFu;

    // Inf / NaN: exponent all-ones.
    if (biased == 0xFFu) {
        if (mant != 0) {
            return (uint16_t)(sign | 0x7E00u);  // NaN -> quiet NaN (nonzero mant)
        }
        return (uint16_t)(sign | 0x7C00u);      // +/- Inf
    }

    int32_t exp = (int32_t)biased - 127 + 15;   // rebias float32 -> float16

    // Overflow of the representable range -> Inf.
    if (exp >= 0x1F) {
        return (uint16_t)(sign | 0x7C00u);
    }

    // Subnormal or underflow (half exponent <= 0).
    if (exp <= 0) {
        if (exp < -10) {
            return (uint16_t)sign;              // too small -> signed zero
        }
        mant |= 0x00800000u;                    // restore implicit leading 1
        int shift = 14 - exp;                   // shift in [14..24]
        uint32_t half_mant = mant >> shift;
        // round-to-nearest-even on the discarded low bits
        uint32_t rem = mant & ((1u << shift) - 1);
        uint32_t halfway = 1u << (shift - 1);
        if (rem > halfway || (rem == halfway && (half_mant & 1u))) {
            half_mant++;                        // may carry up to smallest normal
        }
        return (uint16_t)(sign | half_mant);
    }

    // Normalized.
    uint16_t h = (uint16_t)(sign | ((uint32_t)exp << 10) | (mant >> 13));
    uint32_t rem = mant & 0x1FFFu;              // 13 dropped bits
    if (rem > 0x1000u || (rem == 0x1000u && (h & 1u))) {
        h++;  // round half-to-even; a mantissa carry correctly bumps the exponent
    }
    return h;
}

float FaceDatabase::HalfToFloat32(uint16_t h) {
    uint32_t sign = (uint32_t)(h & 0x8000u) << 16;
    uint32_t exp  = (h >> 10) & 0x1Fu;
    uint32_t mant = h & 0x03FFu;
    uint32_t out;

    if (exp == 0) {
        if (mant == 0) {
            out = sign;                         // +/- zero
        } else {
            // Subnormal half -> normalized float32.
            exp = 127 - 15 + 1;
            while ((mant & 0x0400u) == 0) {
                mant <<= 1;
                exp--;
            }
            mant &= 0x03FFu;                    // drop the leading bit shifted out
            out = sign | (exp << 23) | (mant << 13);
        }
    } else if (exp == 0x1Fu) {
        out = sign | 0x7F800000u | (mant << 13);  // Inf / NaN
    } else {
        out = sign | ((exp - 15 + 127) << 23) | (mant << 13);  // normalized
    }

    float f;
    memcpy(&f, &out, sizeof(f));
    return f;
}

bool FaceDatabase::LoadFromNvs() {
    std::lock_guard<std::mutex> lock(mutex_);

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "Face database not found, starting with empty database");
        loaded_ = true;
        return true;
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
        return false;
    }

    // Format-version gate. v3 stores emb_%d as 256B fp16; any older store wrote
    // 512B float32 under the SAME key name. Reinterpreting those bytes as fp16
    // would be silent corruption, so a db_ver<3 (or absent) store is ignored and
    // treated as an empty DB — the server library re-pushes via batch-update,
    // which rewrites every slot in the new fp16 format and stamps db_ver=3.
    int32_t db_ver = 0;
    if (nvs_get_i32(nvs_handle, "db_ver", &db_ver) != ESP_OK) {
        db_ver = 0;  // legacy v1/v2 had no db_ver, or pre-fp16
    }
    if (db_ver < FACE_DB_FORMAT_VERSION) {
        ESP_LOGW(TAG,
                 "NVS db_ver=%d < %d (pre-fp16 512B format); ignoring as empty, "
                 "awaiting server re-sync",
                 (int)db_ver, FACE_DB_FORMAT_VERSION);
        nvs_close(nvs_handle);
        faces_.clear();
        loaded_ = true;
        return true;
    }

    int32_t count = 0;
    err = nvs_get_i32(nvs_handle, "count", &count);
    if (err != ESP_OK) {
        count = 0;
    }

    ESP_LOGI(TAG, "Loading %d faces from NVS", (int)count);

    faces_.clear();
    for (int i = 0; i < count; i++) {
        FaceEntry entry;
        if (LoadFaceEntry(i, entry)) {
            faces_.push_back(entry);
            ESP_LOGI(TAG, "Loaded face %d: %s", i, entry.name.c_str());
        }
    }

    nvs_close(nvs_handle);
    loaded_ = true;

    ESP_LOGI(TAG, "Face database loaded with %zu entries", faces_.size());
    return true;
}

bool FaceDatabase::SaveToNvs() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for writing: %s", esp_err_to_name(err));
        return false;
    }

    // Save count
    err = nvs_set_i32(nvs_handle, "count", (int32_t)faces_.size());
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save face count: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }

    // Stamp the persistence format version (v2 = per-face subject_id sid_%d key).
    // Best-effort: a missing/old db_ver is treated as legacy v1 on load and is
    // still fully readable (sid_%d simply absent -> sentinel 0).
    nvs_set_i32(nvs_handle, "db_ver", (int32_t)FACE_DB_FORMAT_VERSION);

    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    return err == ESP_OK;
}

bool FaceDatabase::LoadFaceEntry(int index, FaceEntry& entry) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);

    if (err != ESP_OK) {
        return false;
    }

    // Load name
    char name_key[16];
    snprintf(name_key, sizeof(name_key), "name_%d", index);

    size_t name_len = FACE_NAME_MAX_LEN;
    char name_buf[FACE_NAME_MAX_LEN];
    err = nvs_get_str(nvs_handle, name_key, name_buf, &name_len);
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return false;
    }
    entry.name = std::string(name_buf);

    // Load embedding (NVS stores 256B fp16; decode to canonical float32).
    char emb_key[16];
    snprintf(emb_key, sizeof(emb_key), "emb_%d", index);

    uint8_t emb_blob[FACE_EMBEDDING_NVS_SIZE];  // 256B fp16 LE
    size_t emb_len = FACE_EMBEDDING_NVS_SIZE;
    err = nvs_get_blob(nvs_handle, emb_key, emb_blob, &emb_len);
    if (err != ESP_OK || emb_len != FACE_EMBEDDING_NVS_SIZE) {
        ESP_LOGE(TAG, "Failed to load embedding for %s: %s", name_key, esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }
    // Unpack 128 binary16 LE -> float32 (byte-wise: alignment-safe on xtensa).
    for (int i = 0; i < FACE_EMBEDDING_DIM; i++) {
        uint16_t half = (uint16_t)emb_blob[2 * i] |
                        ((uint16_t)emb_blob[2 * i + 1] << 8);
        entry.embedding[i] = HalfToFloat32(half);
    }

    // Load subject_id (v2+). Independent typed key: legacy v1 records simply lack
    // sid_%d, so NVS_NOT_FOUND -> sentinel 0. The embedding blob is strictly
    // length-checked above, so there is no way to misread its tail as a subject_id.
    char sid_key[16];
    snprintf(sid_key, sizeof(sid_key), "sid_%d", index);
    int32_t sid = 0;
    if (nvs_get_i32(nvs_handle, sid_key, &sid) != ESP_OK) {
        sid = 0;
    }
    entry.subject_id = (int)sid;

    nvs_close(nvs_handle);
    return true;
}

bool FaceDatabase::SaveFaceEntry(int index, const FaceEntry& entry) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for writing: %s", esp_err_to_name(err));
        return false;
    }

    // Save name
    char name_key[16];
    snprintf(name_key, sizeof(name_key), "name_%d", index);
    err = nvs_set_str(nvs_handle, name_key, entry.name.c_str());
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save name: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }

    // Save embedding: encode canonical float32 -> 256B fp16 LE blob.
    char emb_key[16];
    snprintf(emb_key, sizeof(emb_key), "emb_%d", index);
    uint8_t emb_blob[FACE_EMBEDDING_NVS_SIZE];
    for (int i = 0; i < FACE_EMBEDDING_DIM; i++) {
        uint16_t half = Float32ToHalf(entry.embedding[i]);
        emb_blob[2 * i]     = (uint8_t)(half & 0xFF);
        emb_blob[2 * i + 1] = (uint8_t)((half >> 8) & 0xFF);
    }
    err = nvs_set_blob(nvs_handle, emb_key, emb_blob, FACE_EMBEDDING_NVS_SIZE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save embedding: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }

    // Save subject_id (v2 format) as an independent i32 key.
    char sid_key[16];
    snprintf(sid_key, sizeof(sid_key), "sid_%d", index);
    err = nvs_set_i32(nvs_handle, sid_key, (int32_t)entry.subject_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save subject_id: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }

    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    return err == ESP_OK;
}

bool FaceDatabase::DeleteFaceEntry(int index) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);

    if (err != ESP_OK) {
        return false;
    }

    char name_key[16];
    snprintf(name_key, sizeof(name_key), "name_%d", index);
    nvs_erase_key(nvs_handle, name_key);

    char emb_key[16];
    snprintf(emb_key, sizeof(emb_key), "emb_%d", index);
    nvs_erase_key(nvs_handle, emb_key);

    char sid_key[16];
    snprintf(sid_key, sizeof(sid_key), "sid_%d", index);
    nvs_erase_key(nvs_handle, sid_key);

    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    return true;
}

bool FaceDatabase::AddFace(const std::string& name, const float* embedding, int subject_id) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (name.empty() || name.length() >= FACE_NAME_MAX_LEN) {
        ESP_LOGE(TAG, "Invalid name length");
        return false;
    }

    if (faces_.size() >= FACE_MAX_COUNT) {
        ESP_LOGE(TAG, "Face database is full (max %d)", FACE_MAX_COUNT);
        return false;
    }

    // Check for duplicate names
    for (const auto& face : faces_) {
        if (face.name == name) {
            ESP_LOGE(TAG, "Face with name '%s' already exists", name.c_str());
            return false;
        }
    }

    FaceEntry entry;
    entry.name = name;
    entry.subject_id = subject_id;
    memcpy(entry.embedding, embedding, FACE_EMBEDDING_SIZE);

    // Normalize the embedding
    NormalizeEmbedding(entry.embedding);

    int index = (int)faces_.size();

    // Save to NVS first
    if (!SaveFaceEntry(index, entry)) {
        ESP_LOGE(TAG, "Failed to save face entry to NVS");
        return false;
    }

    // Add to memory cache
    faces_.push_back(entry);

    // Update count in NVS
    if (!SaveToNvs()) {
        ESP_LOGE(TAG, "Failed to update face count in NVS");
        // Entry is saved, just count update failed
    }

    ESP_LOGI(TAG, "Added face: %s (subject_id=%d, total: %zu)",
             name.c_str(), subject_id, faces_.size());
    return true;
}

bool FaceDatabase::DeleteFace(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);

    for (size_t i = 0; i < faces_.size(); i++) {
        if (faces_[i].name == name) {
            return DeleteFaceByIndexLocked((int)i);
        }
    }

    ESP_LOGW(TAG, "Face not found: %s", name.c_str());
    return false;
}

bool FaceDatabase::DeleteFaceByIndex(int index) {
    std::lock_guard<std::mutex> lock(mutex_);
    return DeleteFaceByIndexLocked(index);
}

bool FaceDatabase::DeleteFaceByIndexLocked(int index) {
    if (index < 0 || index >= (int)faces_.size()) {
        return false;
    }

    std::string name = faces_[index].name;

    // Remove from memory
    faces_.erase(faces_.begin() + index);

    // Rewrite all entries to NVS (to maintain compact indexing)
    // First, delete all old entries
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        // Erase all face entries
        for (int i = 0; i <= (int)faces_.size(); i++) {
            char name_key[16];
            snprintf(name_key, sizeof(name_key), "name_%d", i);
            nvs_erase_key(nvs_handle, name_key);

            char emb_key[16];
            snprintf(emb_key, sizeof(emb_key), "emb_%d", i);
            nvs_erase_key(nvs_handle, emb_key);

            char sid_key[16];
            snprintf(sid_key, sizeof(sid_key), "sid_%d", i);
            nvs_erase_key(nvs_handle, sid_key);
        }
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }

    // Rewrite all remaining entries
    for (size_t i = 0; i < faces_.size(); i++) {
        SaveFaceEntry((int)i, faces_[i]);
    }

    SaveToNvs();

    ESP_LOGI(TAG, "Deleted face: %s (remaining: %zu)", name.c_str(), faces_.size());
    return true;
}

bool FaceDatabase::RenameFace(const std::string& old_name, const std::string& new_name) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (new_name.empty() || new_name.length() >= FACE_NAME_MAX_LEN) {
        ESP_LOGE(TAG, "Invalid new name length");
        return false;
    }

    // Check new name doesn't already exist
    for (const auto& face : faces_) {
        if (face.name == new_name) {
            ESP_LOGE(TAG, "Face with name '%s' already exists", new_name.c_str());
            return false;
        }
    }

    // Find and rename
    for (size_t i = 0; i < faces_.size(); i++) {
        if (faces_[i].name == old_name) {
            faces_[i].name = new_name;
            if (!SaveFaceEntry((int)i, faces_[i])) {
                ESP_LOGE(TAG, "Failed to save renamed entry to NVS");
                faces_[i].name = old_name;  // Rollback
                return false;
            }
            ESP_LOGI(TAG, "Renamed face: %s -> %s", old_name.c_str(), new_name.c_str());
            return true;
        }
    }

    ESP_LOGW(TAG, "Face not found: %s", old_name.c_str());
    return false;
}

std::vector<std::string> FaceDatabase::ListFaces() {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<std::string> names;
    for (const auto& face : faces_) {
        names.push_back(face.name);
    }
    return names;
}

int FaceDatabase::GetFaceCount() {
    std::lock_guard<std::mutex> lock(mutex_);
    return (int)faces_.size();
}

std::vector<FaceEntry> FaceDatabase::GetAllEntries() {
    std::lock_guard<std::mutex> lock(mutex_);
    return faces_;  // copy under lock
}

bool FaceDatabase::ReplaceAll(const std::vector<FaceEntry>& entries) {
    std::lock_guard<std::mutex> lock(mutex_);

    // 1) Build the new generation fully in a LOCAL vector first — validate name
    //    length, copy + normalize the embedding, preserve subject_id, cap at
    //    FACE_MAX_COUNT. faces_ is never touched until the new set is durably
    //    persisted, so Match() (same mutex_) only ever sees old-or-new, never a
    //    half-merged set.
    std::vector<FaceEntry> next;
    next.reserve(entries.size() < (size_t)FACE_MAX_COUNT ? entries.size() : FACE_MAX_COUNT);
    for (const auto& e : entries) {
        if (next.size() >= (size_t)FACE_MAX_COUNT) {
            ESP_LOGW(TAG, "ReplaceAll: input exceeds FACE_MAX_COUNT=%d, truncating", FACE_MAX_COUNT);
            break;
        }
        if (e.name.empty() || e.name.length() >= FACE_NAME_MAX_LEN) {
            ESP_LOGW(TAG, "ReplaceAll: skipping entry with invalid name length");
            continue;
        }
        FaceEntry ne;
        ne.name = e.name;
        ne.subject_id = e.subject_id;
        memcpy(ne.embedding, e.embedding, FACE_EMBEDDING_SIZE);
        NormalizeEmbedding(ne.embedding);
        next.push_back(ne);
    }

    // 2) Persist via a single NVS handle. Strategy = documented FALLBACK (the
    //    16KB NVS partition can't hold two full generations for a slot-flip, and
    //    NVS is not transactional). Crash-degradation: write count=0 + commit
    //    FIRST, so any power loss during the rewrite leaves a CLEAN EMPTY DB
    //    (server is the source of truth → it re-pushes) rather than a corrupt
    //    mix of old and new entries. count=N is written + committed LAST.
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ReplaceAll: nvs_open failed: %s", esp_err_to_name(err));
        return false;
    }

    // 2a) Invalidate first: count=0 committed before any slot is rewritten.
    if (nvs_set_i32(h, "count", 0) != ESP_OK || nvs_commit(h) != ESP_OK) {
        ESP_LOGE(TAG, "ReplaceAll: failed to zero count");
        nvs_close(h);
        return false;
    }

    // 2b) Erase every possible old slot key (old count may exceed the new one,
    //     so stale tail entries must not survive a shrink).
    for (int i = 0; i < FACE_MAX_COUNT; i++) {
        char k[16];
        snprintf(k, sizeof(k), "name_%d", i); nvs_erase_key(h, k);
        snprintf(k, sizeof(k), "emb_%d", i);  nvs_erase_key(h, k);
        snprintf(k, sizeof(k), "sid_%d", i);  nvs_erase_key(h, k);
    }

    // 2c) Write the new generation (name / embedding / subject_id per slot).
    bool write_ok = true;
    for (size_t i = 0; i < next.size(); i++) {
        char k[16];
        snprintf(k, sizeof(k), "name_%d", (int)i);
        if (nvs_set_str(h, k, next[i].name.c_str()) != ESP_OK) { write_ok = false; break; }
        snprintf(k, sizeof(k), "emb_%d", (int)i);
        uint8_t emb_blob[FACE_EMBEDDING_NVS_SIZE];  // float32 -> 256B fp16 LE
        for (int j = 0; j < FACE_EMBEDDING_DIM; j++) {
            uint16_t half = Float32ToHalf(next[i].embedding[j]);
            emb_blob[2 * j]     = (uint8_t)(half & 0xFF);
            emb_blob[2 * j + 1] = (uint8_t)((half >> 8) & 0xFF);
        }
        if (nvs_set_blob(h, k, emb_blob, FACE_EMBEDDING_NVS_SIZE) != ESP_OK) { write_ok = false; break; }
        snprintf(k, sizeof(k), "sid_%d", (int)i);
        if (nvs_set_i32(h, k, (int32_t)next[i].subject_id) != ESP_OK) { write_ok = false; break; }
    }

    if (!write_ok) {
        // Leave count at 0 (committed in 2a) → DB loads empty & clean on reboot.
        ESP_LOGE(TAG, "ReplaceAll: slot write failed; DB left empty for clean re-sync");
        nvs_commit(h);
        nvs_close(h);
        return false;
    }

    // 2d) Publish: stamp the new count + format version, then the final commit.
    nvs_set_i32(h, "db_ver", (int32_t)FACE_DB_FORMAT_VERSION);
    if (nvs_set_i32(h, "count", (int32_t)next.size()) != ESP_OK) {
        ESP_LOGE(TAG, "ReplaceAll: failed to set new count");
        nvs_close(h);
        return false;
    }
    err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ReplaceAll: final nvs_commit failed: %s", esp_err_to_name(err));
        return false;
    }

    // 3) Swap the in-memory DB only after the persist is fully committed. Done
    //    under mutex_, so concurrent Match() transitions atomically old → new.
    faces_ = std::move(next);
    ESP_LOGI(TAG, "ReplaceAll: applied %zu faces", faces_.size());
    return true;
}

FaceMatchResult FaceDatabase::Match(const float* embedding, float threshold) {
    std::lock_guard<std::mutex> lock(mutex_);

    FaceMatchResult result;
    result.matched = false;
    result.name = "";
    result.subject_id = 0;
    result.similarity = 0.0f;
    result.index = -1;

    if (faces_.empty()) {
        return result;
    }

    // Normalize input embedding
    float normalized_emb[FACE_EMBEDDING_DIM];
    memcpy(normalized_emb, embedding, FACE_EMBEDDING_SIZE);
    NormalizeEmbedding(normalized_emb);

    float best_similarity = -1.0f;
    int best_index = -1;

    for (size_t i = 0; i < faces_.size(); i++) {
        float sim = CosineSimilarity(normalized_emb, faces_[i].embedding);
        ESP_LOGD(TAG, "Similarity with %s: %.3f", faces_[i].name.c_str(), sim);

        if (sim > best_similarity) {
            best_similarity = sim;
            best_index = (int)i;
        }
    }

    if (best_index >= 0 && best_similarity >= threshold) {
        result.matched = true;
        result.name = faces_[best_index].name;
        result.subject_id = faces_[best_index].subject_id;
        result.similarity = best_similarity;
        result.index = best_index;
        ESP_LOGD(TAG, "Matched: %s (similarity: %.3f)", result.name.c_str(), result.similarity);
    } else {
        ESP_LOGD(TAG, "No match found (best similarity: %.3f, threshold: %.3f)",
                 best_similarity, threshold);
    }

    return result;
}

std::string FaceDatabase::DecodeName(const std::string& name) {
    if (name.size() > 2 && name[0] == 'u' && name[1] == '_') {
        std::string decoded;
        for (size_t i = 2; i + 1 < name.size(); i += 2) {
            char hi = name[i], lo = name[i + 1];
            auto hex_val = [](char c) -> int {
                if (c >= '0' && c <= '9') return c - '0';
                if (c >= 'a' && c <= 'f') return c - 'a' + 10;
                if (c >= 'A' && c <= 'F') return c - 'A' + 10;
                return -1;
            };
            int h = hex_val(hi), l = hex_val(lo);
            if (h < 0 || l < 0) return name;  // invalid hex, return as-is
            decoded += static_cast<char>((h << 4) | l);
        }
        return decoded;
    }
    return name;
}
