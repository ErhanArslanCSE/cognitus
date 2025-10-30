# cognitus_voice - Improvement Suggestions

Enhancements for the voice interface module while maintaining lightweight design.

---

## High Priority Improvements

### 1. Dedicated Wake Word Detection

**Current:** Post-transcription wake word check (inefficient)

**Improvement:**
- Use dedicated wake word detector (Porcupine, Snowboy, or simple phonetic matcher)
- Run continuously with minimal CPU
- Only activate Whisper when wake word detected

**Benefits:**
- 90% less CPU usage when idle
- Better battery life
- Faster response (no unnecessary transcription)
- Always-on listening feasible

**Implementation:**
```python
# Option 1: Porcupine (lightweight)
import pvporcupine
detector = pvporcupine.create(keywords=['computer'])

# Option 2: Simple phonetic matching
# Match audio patterns without full transcription
```

**Cost:** +5MB model, 1% CPU always-on

---

### 2. Noise Cancellation

**Current:** Raw audio processing

**Improvement:**
- Add noise reduction (noisereduce library)
- Improve accuracy in noisy environments
- Better VAD (voice vs noise separation)

**Implementation:**
```python
import noisereduce as nr
# Reduce noise in audio
cleaned_audio = nr.reduce_noise(
    y=audio_data,
    sr=sample_rate
)
```

**Benefits:**
- Better transcription accuracy
- Works in noisy environments (kitchen, etc.)
- Fewer false activations

**Cost:** +10MB library, +20ms latency

---

### 3. Streaming STT

**Current:** Batch processing (3-second chunks)

**Improvement:**
- Use Whisper streaming mode
- Start transcription as user speaks
- Lower perceived latency

**Benefits:**
- Feels more responsive
- Can interrupt mid-sentence
- Better user experience

**Implementation:**
- Use faster-whisper library (optimized)
- Streaming inference
- Word-by-word output

---

### 4. Voice Cloning for TTS

**Current:** Pre-trained Piper voices

**Improvement:**
- Fine-tune Piper on specific voice
- Personalized robot voice
- Better brand identity

**Benefits:**
- Unique robot voice
- More engaging interaction
- Professional presentation

**Effort:** Medium (requires voice samples and training)

---

### 5. Emotion Detection in Speech

**Current:** No emotion analysis

**Improvement:**
- Detect user emotion from voice (tone, pitch, speed)
- Adjust robot behavior based on emotion
- Respond empathetically

**Use cases:**
- User sounds frustrated → Robot more helpful
- User sounds happy → Robot casual
- User sounds urgent → Robot prioritizes

**Implementation:**
```python
# Simple pitch/energy analysis
pitch = librosa.yin(audio, sr=sample_rate)
energy = np.abs(audio).mean()

if energy > high and pitch > high:
    emotion = "excited"
elif energy < low:
    emotion = "calm"
```

---

## Medium Priority Improvements

### 6. Direction of Arrival (DOA)

**Current:** Single microphone, no direction

**Improvement:**
- Use microphone array (4+ mics)
- Detect sound source direction
- Turn robot toward speaker

**Benefits:**
- Know who's speaking
- Better speaker separation
- Navigate toward user

**Requirements:**
- USB microphone array (ReSpeaker, etc.)
- DOA algorithm (MUSIC, GCC-PHAT)

---

### 7. Speaker Identification

**Current:** No speaker recognition

**Improvement:**
- Identify who is speaking (voice fingerprinting)
- Personalized responses per user
- User-specific preferences

**Implementation:**
```python
from speechbrain.pretrained import SpeakerRecognition
model = SpeakerRecognition.from_hparams("speechbrain/spkrec-ecapa-voxceleb")

speaker_id = model.verify_files(enrollment, test)
```

**Use cases:**
- "Hi John, your keys are..."
- Different personas for different users

---

### 8. Continuous Listening Mode

**Current:** Chunk-based (3-second buffers)

**Improvement:**
- Continuous circular buffer
- Detect sentence boundaries
- Variable-length transcription

**Benefits:**
- More natural conversation
- Handle long sentences
- Better context

---

### 9. TTS Emotion/Prosody Control

**Current:** Neutral voice only

**Improvement:**
- Add emotion to speech (happy, sad, urgent, calm)
- Adjust prosody based on context
- More expressive communication

**Implementation:**
```python
# Piper supports some prosody control
# Adjust speaking rate, pitch for emotion

if context == "emergency":
    rate = 1.3  # Speak faster
    pitch = 1.1  # Higher pitch
```

---

### 10. Audio Classification

**Current:** Only amplitude monitoring

**Improvement:**
- Classify sound types (door knock, alarm, glass breaking)
- Use simple CNN model (YAMNet, PANNs)
- Provide context to brain

**Benefits:**
- "I heard a door knock"
- "Glass breaking sound detected"
- Better situation awareness

**Model:** YAMNet (5MB, TensorFlow Lite)

---

## Low Priority / Future

### 11. Multi-speaker Conversation

**Current:** Single-user interaction

**Improvement:**
- Handle multiple speakers
- Separate conversations
- Direct responses appropriately

---

### 12. Voice Activity Detection (VAD) Enhancement

**Current:** Simple energy-based VAD

**Improvement:**
- Use WebRTC VAD (more robust)
- ML-based VAD (silero-vad)
- Better speech/noise separation

**Benefits:**
- Fewer false activations
- Better in noisy environments
- Lower power consumption

---

### 13. Conversation State Management

**Current:** Stateless (each command independent)

**Improvement:**
- Track conversation state
- Handle follow-up questions
- Maintain context

**Example:**
```
User: "Where are my keys?"
Robot: "On the table"
User: "Which table?"  ← Needs context
Robot: "The coffee table in the living room"
```

---

### 14. Multi-modal TTS

**Current:** Voice only

**Improvement:**
- Display text on screen simultaneously
- Show visual feedback during speech
- Helpful for hearing impaired

---

### 15. Voice Command Shortcuts

**Current:** Natural language only

**Improvement:**
- Add quick command phrases
- "Status" → Full system status
- "Follow me" → Activates following behavior

**Benefits:**
- Faster interaction
- Less cognitive load
- Power user features

---

## Research Opportunities

### Integration with Academic Papers

**Paper 4: Spatial Language Grounding**
- Enhance TTS to produce natural spatial descriptions
- "On the table, next to your phone"
- Uncertainty expressions: "I think it might be..."

**Paper 8: Multi-Persona Adaptation**
- Different voices for different personas
- Adjust speaking style based on context
- Guardian mode: Alert tone
- Companion mode: Friendly tone

---

## Implementation Priority

**Immediate (Week 1-2):**
1. Dedicated wake word detection (Porcupine)
2. Noise cancellation (noisereduce)

**Short-term (Month 1):**
3. Streaming STT (faster-whisper)
4. Better VAD (WebRTC VAD)

**Medium-term (Month 2-3):**
5. Audio classification (YAMNet)
6. Direction of arrival (mic array)
7. TTS emotion control

**Long-term (Month 3+):**
8. Speaker identification
9. Voice cloning
10. Multi-speaker handling

---

## Testing Recommendations

### Unit Tests
- Audio capture (mock PyAudio)
- Whisper transcription (sample audio files)
- Piper synthesis (text to WAV)
- Wake word detection logic

### Integration Tests
- End-to-end STT→Brain→TTS
- Audio monitor event generation
- Multi-language transcription
- Error recovery

### Performance Tests
- STT latency measurement
- TTS latency measurement
- Memory usage profiling
- CPU usage monitoring
- Battery impact assessment

---

## Known Limitations

1. **Post-transcription wake word** - CPU waste when not activated
2. **No noise cancellation** - Poor performance in loud environments
3. **Batch processing** - 3-second delay for transcription
4. **No beamforming** - Single mic, no directionality
5. **Limited error recovery** - Model failures may require restart
6. **No online learning** - Can't adapt to user's voice over time

---

## Hardware Recommendations

### Current Setup:
- Generic USB microphone (works but basic)

### Recommended Upgrades:
1. **ReSpeaker 4-Mic Array** (~$30)
   - Better pickup range
   - Direction of arrival
   - Noise cancellation
   - Built-in wake word

2. **Better speakers**
   - Current: Generic USB/3.5mm
   - Upgrade: Directional speakers
   - Better audio quality

---

## Alternative Implementations

### For Lower Latency:
- Replace Whisper with Vosk (faster, less accurate)
- Use streaming ASR (Google/Azure)
- Optimize Whisper with ONNX Runtime

### For Better Quality:
- Use Whisper Base instead of Tiny (+accuracy, -speed)
- Use larger Piper voices (+quality, +size)

### For Offline:
- Current implementation IS offline ✓
- All processing on-device
- No internet needed

---

## Maintenance Notes

### Model Updates
- Whisper: Updated by OpenAI (check for new versions)
- Piper: New voices released regularly

### Configuration Tuning
- Adjust `vad_threshold` based on environment
- Tune `wake_words` for false positive rate
- Adjust `speaking_rate` for user preference

### Monitoring
- Check `/voice/stt_status` for transcription quality
- Monitor `/audio/level` for ambient noise
- Watch CPU usage (should be <30%)

---

**Last updated:** 2024-10-30
**Status:** Complete and functional
**Next milestone:** Dedicated wake word detector
