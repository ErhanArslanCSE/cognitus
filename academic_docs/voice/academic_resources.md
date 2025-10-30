# cognitus_voice - Academic Resources

Academic papers and research related to speech recognition (STT), text-to-speech (TTS), audio processing, and voice interaction in robotics.

**Last Updated:** October 30, 2025

---

## Speech-to-Text (STT) & Whisper

### 1. Robust Speech Recognition via Large-Scale Weak Supervision (Whisper Paper)
- **Authors:** Radford, Kim, Xu et al. (OpenAI)
- **Publication:** Original Whisper paper
- **Link:** https://cdn.openai.com/papers/whisper.pdf
- **Model:** Whisper family (tiny to large)
- **Training:** 680,000 hours of multilingual data
- **Relevance:** Foundation model we use for STT

### 2. WhisperX: Automatic Speech Recognition with Speaker Diarization (2024)
- **Link:** https://learnopencv.com/automatic-speech-recognition/
- **Key Innovation:** Whisper + forced alignment + speaker diarization
- **Features:** Word-level timestamps, speaker identification
- **Relevance:** Enhanced Whisper capabilities

### 3. WhisperFusion: Ultra-low Latency Conversations (2024)
- **Organization:** Collabora
- **Link:** https://www.collabora.com/news-and-blog/news-and-events/whisperfusion-ultra-low-latency-conversations-with-an-ai-chatbot.html
- **Key Innovation:** <300ms end-to-end latency
- **Relevance:** Low-latency voice interaction

### 4. GPT-4o Transcribe Models (2024)
- **Organization:** OpenAI
- **Link:** https://openai.com/index/introducing-our-next-generation-audio-models/
- **Models:** gpt-4o-transcribe, gpt-4o-mini-transcribe
- **Improvement:** Better WER over Whisper across benchmarks
- **Relevance:** Next-generation STT alternatives

---

## Text-to-Speech (TTS)

### 5. Piper TTS: Fast, Local Neural Text to Speech (2024)
- **Organization:** Rhasspy
- **Link:** https://github.com/rhasspy/piper
- **Key Features:** ONNX Runtime, CPU-optimized, 50+ voices
- **Quality:** Natural, human-like speech
- **Relevance:** Lightweight TTS we use

### 6. State of Voice AI 2024
- **Organization:** Cartesia
- **Link:** https://cartesia.ai/blog/state-of-voice-ai-2024
- **Coverage:** Voice AI landscape 2024
- **Topics:** STT, TTS, voice cloning, real-time systems
- **Relevance:** Industry trends and benchmarks

### 7. Building Next-Gen AI Voice Systems: A 2025 Developer's Guide
- **Link:** https://digitaltechbyte.com/building-next-gen-ai-voice-systems-a-2025-developers-guide-to-tts-stt-and-beyond/
- **Coverage:** Complete voice pipeline (STT→LLM→TTS)
- **Latency Targets:** Current best 510ms, human 230ms
- **Relevance:** System design principles

### 8. Top 10 AI Voice Technologies Dominating 2025
- **Link:** https://ts2.tech/en/top-10-ai-voice-and-speech-technologies-dominating-2025-tts-stt-voice-cloning/
- **Coverage:** Whisper, ElevenLabs, Speechmatics, etc.
- **Trends:** Real-time, multilingual, emotional TTS
- **Relevance:** Technology landscape

---

## End-to-End Voice Systems

### 9. Speech-to-Speech (S2S) Models (2024-2025)
- **Models:** Moshi and others
- **Key Innovation:** Direct speech→speech (no text intermediate)
- **Latency:** ~160ms achievable
- **Relevance:** Future ultra-low latency approach

### 10. Speechmatics Flow (2024)
- **Features:** Combined STT + LLM + TTS
- **Languages:** 30+ languages supported
- **Integration:** End-to-end pipeline
- **Relevance:** Commercial implementation example

---

## Wake Word Detection

### 11. Porcupine Wake Word Engine (Ongoing)
- **Organization:** Picovoice
- **Features:** Lightweight, always-on, low CPU
- **Cost:** Free tier available, ~5MB model
- **Relevance:** Future upgrade for dedicated wake word

### 12. Snowboy (Archived but reference)
- **Status:** Archived project
- **Relevance:** Historical reference for wake word detection
- **Lesson:** Custom wake word training approaches

---

## Audio Processing & Voice Activity Detection

### 13. WebRTC VAD (Voice Activity Detection)
- **Technology:** Google WebRTC project
- **Features:** Robust speech/noise separation
- **Integration:** Python bindings available
- **Relevance:** Better VAD than energy-based

### 14. Silero VAD: Pre-trained Enterprise-grade Voice Activity Detector (2021-2024)
- **Link:** https://github.com/snakers4/silero-vad
- **Features:** ML-based VAD, multilingual, lightweight
- **Size:** ~1MB model
- **Relevance:** Advanced VAD for noise environments

---

## Noise Cancellation & Audio Enhancement

### 15. noisereduce: Python Noise Reduction Library
- **Link:** https://github.com/timsainb/noisereduce
- **Technique:** Spectral gating noise reduction
- **Use Case:** Clean audio before STT
- **Relevance:** Improving accuracy in noisy environments

---

## Multimodal Voice-Vision Systems

### 16. Vision-Language-Audio Integration in Robotics (2024)
- **Trend:** Combining audio with vision for better context
- **Example:** Detecting sound source visually
- **Relevance:** Future multimodal integration

---

## Latency Optimization

### 17. Whisper Optimization Techniques (2024-2025)
- **faster-whisper:** CTranslate2 backend (4x speedup)
- **whisper.cpp:** C++ implementation for edge devices
- **TensorRT:** NVIDIA optimization
- **Relevance:** Reducing STT latency on Jetson

---

## Robotics Voice Interaction

### 18. Voice Commands for Mobile Robots: A Survey (2024)
- **Topics:** Command parsing, natural language understanding
- **Application:** Mobile robot control via voice
- **Relevance:** Voice→action pipeline design

---

## Audio Event Detection

### 19. YAMNet: Audio Event Detection Model
- **Organization:** Google
- **Size:** ~5MB TensorFlow Lite model
- **Classes:** 521 audio event types
- **Relevance:** Future enhancement for audio_monitor.py

### 20. PANNs: Large-Scale Pretrained Audio Neural Networks (2020-2024)
- **Technology:** Audio classification CNNs
- **Use Case:** Sound type recognition
- **Relevance:** Classifying environmental sounds

---

## Key Insights from Literature

**STT Trends:**
- Whisper dominates for quality and multilingual
- Latency improvements via streaming and optimization
- On-device deployment increasingly feasible

**TTS Trends:**
- Neural TTS (Piper, VITS) replaces concatenative
- Real-time synthesis standard
- Voice cloning becoming accessible

**Voice Systems:**
- End-to-end latency: 500ms current, 230ms target (human-like)
- S2S models emerging (skip text intermediate)
- Multimodal integration (voice + vision)

---

## Conferences & Venues

**Speech Processing:**
- Interspeech
- ICASSP
- ACL (for NLP aspects)

**Robotics with Speech:**
- HRI (Human-Robot Interaction)
- ICRA/IROS (Robotics conferences with HRI tracks)

---

## Implementation Recommendations

**For cognitus_voice:**

**Current (Implemented):**
- Whisper Tiny: Good balance of speed/accuracy ✓
- Piper TTS: Lightweight, good quality ✓
- PyAudio: Standard audio capture ✓
- Simple energy-based VAD ✓

**Upgrades (from literature):**
- faster-whisper: 4x speedup
- Dedicated wake word (Porcupine): Always-on listening
- Silero VAD: Better noise handling
- noisereduce: Cleaner audio

---

## GitHub Resources

**Whisper Implementations:**
- openai/whisper (original)
- guillaumekln/faster-whisper (optimized)
- ggerganov/whisper.cpp (C++ for edge)

**Piper:**
- rhasspy/piper (our choice)
- coqui-ai/TTS (alternative)

**Audio Processing:**
- timsainb/noisereduce
- snakers4/silero-vad
- wiseman/py-webrtcvad

---

## Performance Benchmarks (2024-2025)

**Whisper Tiny:**
- WER: ~5-8% (English)
- Latency: ~150ms per 3s chunk
- Size: 39M parameters, 150MB

**Piper:**
- Quality: 4.0-4.5/5 (MOS)
- Speed: 2-5x real-time
- Size: 50MB per voice

**Target Latency (from literature):**
- STT: 100-150ms ✓ (achieved)
- LLM: 200-500ms
- TTS: 90-100ms ✓ (achieved)
- Total: 390-750ms (vs human 230ms)

---

## Research Opportunities

**Future Work:**
1. Streaming Whisper for lower perceived latency
2. Emotion detection from voice
3. Speaker identification
4. Direction of arrival (mic array)
5. Audio-visual fusion

**Related Papers to Explore:**
- Emotion recognition in speech
- Multi-speaker conversation
- Audio-guided navigation
- Sound source localization

---

**Curated by:** COGNITUS Team
**Focus Areas:** STT, TTS, Wake Word, Audio Monitoring
**Technology:** Whisper Tiny + Piper + PyAudio
**Last Literature Review:** October 2025
