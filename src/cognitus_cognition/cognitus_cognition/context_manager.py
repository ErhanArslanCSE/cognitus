"""
Context Manager - Builds context for LLM
Combines scene info, conversation history, memory retrieval
"""

from datetime import datetime


class ContextManager:
    """
    Manages context for LLM inference
    - Current scene (from perception)
    - Conversation history
    - Retrieved memories
    - System prompt
    """

    def __init__(self, system_prompt=None, max_history=10):
        """
        Args:
            system_prompt: System role description
            max_history: Maximum conversation turns to keep
        """
        self.system_prompt = system_prompt or self._default_system_prompt()
        self.max_history = max_history

        # State
        self.conversation_history = []
        self.current_scene = "unknown"
        self.current_location = "unknown"
        self.recent_memories = []

    def _default_system_prompt(self):
        """Default system prompt"""
        return """You are COGNITUS, an autonomous robot with:
- Vision: You can see objects and understand spatial relationships
- Memory: You remember past events and can recall them
- Reasoning: You can make decisions and answer questions
- Actions: You can move and perform tasks

Be helpful, concise (2-3 sentences), and reference what you observe.
Use spatial descriptions when relevant (on, near, left of, etc.).
Admit uncertainty when appropriate."""

    def update_scene(self, scene_description):
        """Update current scene from perception"""
        self.current_scene = scene_description

    def update_location(self, location):
        """Update current location"""
        self.current_location = location

    def add_memory(self, memory_text):
        """Add retrieved memory to context"""
        self.recent_memories.append(memory_text)
        # Keep only recent memories
        if len(self.recent_memories) > 5:
            self.recent_memories = self.recent_memories[-5:]

    def add_turn(self, user_input, assistant_response):
        """Add conversation turn to history"""
        self.conversation_history.append({
            'user': user_input,
            'assistant': assistant_response,
            'timestamp': datetime.now().isoformat()
        })

        # Keep only recent history
        if len(self.conversation_history) > self.max_history:
            self.conversation_history = self.conversation_history[-self.max_history:]

    def build_prompt(self, user_query):
        """
        Build complete prompt for LLM

        Returns:
            Full prompt string with context
        """
        prompt_parts = []

        # System prompt
        prompt_parts.append(f"<|system|>\n{self.system_prompt}\n<|end|>")

        # Current context
        context_parts = []

        if self.current_scene != "unknown":
            context_parts.append(f"Current observation: {self.current_scene}")

        if self.current_location != "unknown":
            context_parts.append(f"Location: {self.current_location}")

        context_parts.append(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M')}")

        if self.recent_memories:
            context_parts.append(f"Relevant memories: {'; '.join(self.recent_memories[-3:])}")

        prompt_parts.append(f"\n<|context|>\n{' | '.join(context_parts)}\n<|end|>")

        # Conversation history
        if self.conversation_history:
            history_parts = []
            for turn in self.conversation_history[-5:]:  # Last 5 turns
                history_parts.append(f"User: {turn['user']}")
                history_parts.append(f"Assistant: {turn['assistant']}")

            prompt_parts.append(f"\n<|history|>\n{chr(10).join(history_parts)}\n<|end|>")

        # Current query
        prompt_parts.append(f"\n<|user|>\n{user_query}\n<|end|>")
        prompt_parts.append(f"\n<|assistant|>\n")

        return "\n".join(prompt_parts)

    def clear_history(self):
        """Clear conversation history"""
        self.conversation_history = []

    def clear_memories(self):
        """Clear retrieved memories"""
        self.recent_memories = []

    def get_context_stats(self):
        """Get context statistics"""
        return {
            'history_length': len(self.conversation_history),
            'memories_count': len(self.recent_memories),
            'current_scene': self.current_scene != "unknown",
            'current_location': self.current_location != "unknown"
        }
