#ifndef FACE_SERIAL_HANDLER_H
#define FACE_SERIAL_HANDLER_H

#include <esp_console.h>

// Registers face database CRUD commands with the ESP console.
// Commands: face_list, face_add, face_delete, face_rename,
//           inference_pause, inference_resume
// Responses are JSON printed to stdout (visible on USB secondary console).
class FaceSerialHandler {
public:
    static FaceSerialHandler& GetInstance();
    void RegisterCommands();

private:
    FaceSerialHandler() = default;
    ~FaceSerialHandler() = default;
    FaceSerialHandler(const FaceSerialHandler&) = delete;
    FaceSerialHandler& operator=(const FaceSerialHandler&) = delete;

    static int CmdList(int argc, char** argv);
    static int CmdAdd(int argc, char** argv);
    static int CmdDelete(int argc, char** argv);
    static int CmdRename(int argc, char** argv);
    static int CmdExport(int argc, char** argv);
    static int CmdInferencePause(int argc, char** argv);
    static int CmdInferenceResume(int argc, char** argv);
    static int CmdBench(int argc, char** argv);  // path-Z single-shot latency probe
};

#endif // FACE_SERIAL_HANDLER_H
