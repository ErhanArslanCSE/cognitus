"""
LLM Interface - Generic LLM loader and inference
Supports multiple models via config:
  - BitNet b1.58 (1-bit, ultra-lightweight)
  - Phi-3.5-mini (4-bit, high capability)
  - Any HuggingFace transformers model
"""

import torch
from transformers import AutoTokenizer, AutoModelForCausalLM, BitsAndBytesConfig


class LLMInterface:
    """
    Generic LLM interface
    Loads any model from config, provides unified inference API
    """

    def __init__(self, model_name, quantization='4bit', max_tokens=150, temperature=0.7, logger=None):
        """
        Args:
            model_name: HuggingFace model ID (e.g., "microsoft/Phi-3.5-mini-instruct")
            quantization: "1bit", "4bit", "8bit", or "none"
            max_tokens: Maximum generation tokens
            temperature: Sampling temperature
            logger: ROS logger (optional)
        """
        self.model_name = model_name
        self.quantization = quantization
        self.max_tokens = max_tokens
        self.temperature = temperature
        self.logger = logger

        self.tokenizer = None
        self.model = None
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        self._log(f'Loading LLM: {model_name}')
        self._log(f'  Quantization: {quantization}')
        self._log(f'  Device: {self.device}')

        self._load_model()

    def _log(self, msg):
        """Log message"""
        if self.logger:
            self.logger.info(msg)
        else:
            print(msg)

    def _load_model(self):
        """Load model and tokenizer"""
        try:
            # Load tokenizer
            self.tokenizer = AutoTokenizer.from_pretrained(
                self.model_name,
                trust_remote_code=True
            )

            # Configure quantization
            quantization_config = None

            if self.quantization == '4bit':
                quantization_config = BitsAndBytesConfig(
                    load_in_4bit=True,
                    bnb_4bit_quant_type='nf4',
                    bnb_4bit_compute_dtype=torch.float16,
                    bnb_4bit_use_double_quant=True
                )
                self._log('  Using 4-bit quantization (NF4)')

            elif self.quantization == '8bit':
                quantization_config = BitsAndBytesConfig(
                    load_in_8bit=True
                )
                self._log('  Using 8-bit quantization')

            elif self.quantization == '1bit':
                # BitNet requires special handling
                self._log('  BitNet 1-bit mode')
                # Note: BitNet needs bitnet.cpp for optimal performance
                # For now, load normally and warn
                self._log('  ⚠ For optimal BitNet performance, use bitnet.cpp')

            # Load model
            self.model = AutoModelForCausalLM.from_pretrained(
                self.model_name,
                quantization_config=quantization_config,
                device_map='auto',
                trust_remote_code=True,
                torch_dtype=torch.float16 if self.device == 'cuda' else torch.float32
            )

            self._log(f'✓ Model loaded: {self.model_name}')
            self._log(f'  Parameters: {self._count_parameters()}')
            self._log(f'  Memory: {self._estimate_memory():.1f}GB')

        except ImportError as e:
            self._log(f'❌ Missing dependency: {e}')
            self._log('  Install: pip install transformers bitsandbytes accelerate')
            raise

        except Exception as e:
            self._log(f'❌ Model load failed: {e}')
            raise

    def generate(self, prompt, max_new_tokens=None, temperature=None):
        """
        Generate response from prompt

        Args:
            prompt: Input text
            max_new_tokens: Override default max tokens
            temperature: Override default temperature

        Returns:
            Generated text (str)
        """
        if self.model is None or self.tokenizer is None:
            return "LLM not loaded"

        try:
            # Tokenize
            inputs = self.tokenizer(prompt, return_tensors='pt')
            inputs = {k: v.to(self.device) for k, v in inputs.items()}

            # Generate
            with torch.no_grad():
                outputs = self.model.generate(
                    **inputs,
                    max_new_tokens=max_new_tokens or self.max_tokens,
                    temperature=temperature or self.temperature,
                    do_sample=True,
                    top_p=0.9,
                    repetition_penalty=1.1,
                    pad_token_id=self.tokenizer.eos_token_id
                )

            # Decode
            generated_text = self.tokenizer.decode(outputs[0], skip_special_tokens=True)

            # Remove prompt from output (get only new generation)
            generated_text = generated_text[len(prompt):].strip()

            return generated_text

        except Exception as e:
            self._log(f'Generation error: {e}')
            return f"Error: {str(e)}"

    def _count_parameters(self):
        """Count model parameters"""
        try:
            total = sum(p.numel() for p in self.model.parameters())
            if total > 1e9:
                return f"{total/1e9:.1f}B"
            elif total > 1e6:
                return f"{total/1e6:.1f}M"
            else:
                return f"{total/1e3:.1f}K"
        except:
            return "unknown"

    def _estimate_memory(self):
        """Estimate GPU memory usage"""
        try:
            if self.device == 'cuda':
                return torch.cuda.memory_allocated() / 1e9
            else:
                return 0.0
        except:
            return 0.0

    def unload(self):
        """Unload model from memory"""
        if self.model:
            del self.model
            self.model = None

        if torch.cuda.is_available():
            torch.cuda.empty_cache()

        self._log('Model unloaded')
