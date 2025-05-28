from ecm.tools.stt.engine import STT

if __name__ == "__main__":
    stt = STT()
    input("Press Enter to start STT...")
    stt.start()
    input("Press Enter to stop STT...")
    stt.stop()
    print("Transcription:", stt.text)
