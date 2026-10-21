#ifndef FACE_DATABASE_H
#define FACE_DATABASE_H

#include <string>
#include <vector>
#include <mutex>
#include <cstdint>

// Face embedding dimension (MobileFaceNet output)
#define FACE_EMBEDDING_DIM 128
// CANONICAL in-memory size: faces_ and every embedding[] is float32. The wire
// (HTTP/MCP input) and the NVS store are the only two places where a smaller
// encoding (fp16, future int8) appears; everything in RAM stays float32.
#define FACE_EMBEDDING_SIZE (FACE_EMBEDDING_DIM * sizeof(float))  // 512 bytes (RAM)
// NVS storage size: each emb_%d blob is 128 IEEE-754 binary16 (fp16) = 256B.
// Half the float32 footprint — the 16KB NVS partition is the constraint. fp16
// only lives on the NVS/wire boundary; load/save convert to/from float32.
#define FACE_EMBEDDING_NVS_SIZE (FACE_EMBEDDING_DIM * sizeof(uint16_t))  // 256 bytes (NVS)
#define FACE_NAME_MAX_LEN 32
#define FACE_MAX_COUNT 20

// Persistence format version.
//   v1 (implicit / no "db_ver"): name_%d + emb_%d (512B float32) only.
//   v2: adds an independent sid_%d (subject_id) key; emb_%d still 512B float32.
//   v3: emb_%d is now 256B fp16 (binary16 LE). faces_ remains float32 in RAM;
//       load decodes fp16->float32, save encodes float32->fp16.
// Because the emb_%d *byte width* changed (512->256) between v2 and v3, a v3
// load MUST treat any db_ver<3 store as incompatible and ignore it as an empty
// DB (server re-syncs the source-of-truth library). Never reinterpret an old
// 512B blob as 256B fp16.
#define FACE_DB_FORMAT_VERSION 3

struct FaceEntry {
    std::string name;
    int subject_id = 0;  // warehouse subject id; 0 = unknown / not set
    float embedding[FACE_EMBEDDING_DIM];
};

struct FaceMatchResult {
    bool matched;
    std::string name;
    int subject_id;  // warehouse subject id of matched face; 0 if not matched/unknown
    float similarity;
    int index;  // Index in database, -1 if not matched
};

class FaceDatabase {
public:
    static FaceDatabase& GetInstance();

    // Database operations
    // subject_id: warehouse subject id persisted alongside the face (0 = unknown).
    bool AddFace(const std::string& name, const float* embedding, int subject_id = 0);
    bool DeleteFace(const std::string& name);
    bool DeleteFaceByIndex(int index);
    bool RenameFace(const std::string& old_name, const std::string& new_name);
    std::vector<std::string> ListFaces();
    int GetFaceCount();

    // Get a snapshot of all entries (name + embedding) for SBC-side matching export
    std::vector<FaceEntry> GetAllEntries();

    // Wholesale-replace the entire DB with `entries` (server library full sync).
    // Atomic w.r.t. Match(): the whole op runs under mutex_ and the in-memory
    // faces_ is swapped only after a successful batched NVS persist, so Match()
    // never observes a half-updated window. subject_id is preserved per entry.
    // NOTE on durability: the 16KB NVS partition can't hold two full generations,
    // and NVS is not transactional, so this uses a single-namespace batched write
    // (count zeroed first → an interrupted write degrades to a clean EMPTY DB that
    // the server re-syncs, never a silently corrupt match set). Returns true on a
    // fully committed replace.
    bool ReplaceAll(const std::vector<FaceEntry>& entries);

    // Face matching
    FaceMatchResult Match(const float* embedding, float threshold = 0.4f);

    // Utility
    static float CosineSimilarity(const float* a, const float* b, int dim = FACE_EMBEDDING_DIM);
    static void NormalizeEmbedding(float* embedding, int dim = FACE_EMBEDDING_DIM);

    // IEEE-754 binary16 <-> float32 software conversion. Used only at the two
    // narrow encodings boundaries (NVS blob + HTTP/MCP wire); the in-memory
    // embedding stays float32. Full handling of subnormal / inf / nan / overflow
    // with round-to-nearest-even. Public so the HTTP batch-update handler can
    // decode an fp16 wire payload into canonical float32 before ReplaceAll.
    static uint16_t Float32ToHalf(float f);
    static float    HalfToFloat32(uint16_t h);

    // Decode hex-encoded UTF-8 names (e.g. "u_e88b8fe7a6be" -> "苏禾")
    static std::string DecodeName(const std::string& name);

private:
    bool DeleteFaceByIndexLocked(int index);

    FaceDatabase();
    ~FaceDatabase() = default;

    // Disable copy
    FaceDatabase(const FaceDatabase&) = delete;
    FaceDatabase& operator=(const FaceDatabase&) = delete;

    // NVS operations
    bool LoadFromNvs();
    bool SaveToNvs();
    bool LoadFaceEntry(int index, FaceEntry& entry);
    bool SaveFaceEntry(int index, const FaceEntry& entry);
    bool DeleteFaceEntry(int index);

    // In-memory cache
    std::vector<FaceEntry> faces_;
    bool loaded_;
    std::mutex mutex_;
};

#endif // FACE_DATABASE_H
