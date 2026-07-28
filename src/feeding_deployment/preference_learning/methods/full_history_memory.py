from __future__ import annotations

from typing import List, Optional

# Character size at which get_memory_block starts warning (~4 chars/token, so
# ~50k tokens). The block is still returned in full -- the "single_full_history"
# baseline is deliberately literal ("store everything, give it all to the LLM");
# the warning just keeps a long deployment from silently creeping toward the
# model's context limit.
_WARN_BLOCK_CHARS = 200_000


class FullHistoryMemoryModel:
    """Verbatim single-layer memory: every prior finalized meal's episode text,
    in chronological order. No summarization, no embeddings, no retrieval --
    prediction gets the complete history. This is the "single_full_history"
    backend of PredictionModel (research baseline against the three-layer
    semantic/episodic/working split)."""

    def __init__(self, max_days: Optional[int] = None) -> None:
        # When set, get_memory_block keeps only the most recent max_days
        # episodes (guard against unbounded prompt growth on long deployments).
        self.max_days = max_days
        self._episode_texts: List[str] = []

    def load_history(self, episode_texts: List[str]) -> None:
        """Seed the history from persisted prior-day episode texts (chronological)."""
        self._episode_texts = list(episode_texts)

    def add_episode(self, episode_text: str) -> None:
        self._episode_texts.append(episode_text)

    def get_memory_block(self) -> str:
        """All stored episodes joined into one prompt block ("" when empty)."""
        texts = self._episode_texts
        if self.max_days is not None and self.max_days >= 0:
            texts = texts[-self.max_days:] if self.max_days else []
        block = "\n\n".join(texts)
        if len(block) > _WARN_BLOCK_CHARS:
            print(
                f"Warning: full-history memory block is {len(block)} characters "
                f"(~{len(block) // 4} tokens) across {len(texts)} episodes; "
                f"consider setting max_days before this hits the model's context limit.",
                flush=True,
            )
        return block

    def reset(self) -> None:
        self._episode_texts = []
