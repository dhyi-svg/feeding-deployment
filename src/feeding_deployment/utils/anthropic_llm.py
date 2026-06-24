"""Anthropic (Claude) implementation of tomsutils' LargeLanguageModel.

Drop-in replacement for ``tomsutils.llm.OpenAILLM`` for text-only chat use.
It reuses tomsutils' disk-caching ``sample_completions()`` unchanged; only the
provider call inside ``_sample_completions()`` differs, so every existing
``self.llm.sample_completions(...)`` call site keeps working as-is.

``LargeLanguageModel`` is itself a subclass of ``PretrainedLargeModel`` (the
text-only LLM base that ``OpenAILLM`` also uses), so this satisfies the
``PretrainedLargeModel`` interface while giving us the same surface OpenAILLM
exposed (including ``.query()`` and the images-not-supported guard).

Requires the ``ANTHROPIC_API_KEY`` environment variable, which the anthropic
SDK reads automatically. Add ``anthropic`` to the project dependencies.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import anthropic
import PIL.Image
from tomsutils.llm import LargeLanguageModel

# Opus 4.7/4.8 and the Fable/Mythos family removed the sampling parameters
# (temperature / top_p / top_k); sending temperature to them returns HTTP 400.
# Only forward temperature on models that still accept it (Opus 4.6 and older,
# Sonnet, Haiku).
_NO_SAMPLING_PARAM_PREFIXES = (
    "claude-opus-4-8",
    "claude-opus-4-7",
    "claude-fable-5",
    "claude-mythos-5",
)


class AnthropicLLM(LargeLanguageModel):
    """Interface to Anthropic Claude chat models.

    Mirrors the constructor of ``tomsutils.llm.OpenAILLM`` so existing call
    sites can switch by changing only the class name. Assumes
    ``ANTHROPIC_API_KEY`` is set in the environment.
    """

    def __init__(
        self,
        model_name: str = "claude-opus-4-8",
        cache_dir: Path = Path("llm_cache"),
        max_tokens: int = 700,
        use_cache_only: bool = False,
    ) -> None:
        super().__init__(cache_dir, use_cache_only)
        self._model_name = model_name
        # max_tokens is the maximum response length and is REQUIRED by the
        # Anthropic Messages API (unlike OpenAI, where it is optional).
        self._max_tokens = max_tokens
        # The anthropic SDK reads ANTHROPIC_API_KEY from the environment.
        self._client = anthropic.Anthropic()

    def get_id(self) -> str:
        # Distinct from OpenAILLM's "openai-..." id, so the on-disk cache will
        # not return stale OpenAI completions for the same prompt.
        return f"anthropic-{self._model_name}"

    def _sample_completions(
        self,
        prompt: str,
        imgs: list[PIL.Image.Image] | None,
        temperature: float,
        seed: int,
        num_completions: int = 1,
    ) -> tuple[list[str], dict[str, Any]]:
        assert imgs is None, "AnthropicLLM is text-only; use a VLM for images."
        assert num_completions == 1, "Only num_completions=1 is supported."
        # Anthropic has no message-level `seed`; determinism is not guaranteed
        # (it never was via the OpenAI seed either). seed is still part of the
        # tomsutils cache key, so caching behavior is unchanged.
        kwargs: dict[str, Any] = {
            "model": self._model_name,
            "max_tokens": self._max_tokens,
            "messages": [{"role": "user", "content": prompt}],
        }
        if not self._model_name.startswith(_NO_SAMPLING_PARAM_PREFIXES):
            kwargs["temperature"] = temperature
        response = self._client.messages.create(**kwargs)
        # Concatenate any text blocks (a refusal yields no text block, leaving
        # an empty string — callers see "" rather than an index error).
        text = "".join(b.text for b in response.content if b.type == "text")
        metadata = {
            "input_tokens": response.usage.input_tokens,
            "output_tokens": response.usage.output_tokens,
            "model": response.model,
            "stop_reason": response.stop_reason,
        }
        return [text], metadata

    def get_multiple_choice_logprobs(
        self, prompt: str, choices: list[str], seed: int
    ) -> tuple[dict[str, float], dict[str, Any]]:
        # The Anthropic API does not expose token logprobs. Nothing in this
        # codebase calls this on the wrapper, but it is abstract on
        # PretrainedLargeModel and must be defined for the class to instantiate.
        raise NotImplementedError(
            "Claude has no logprobs API; get_multiple_choice_logprobs is "
            "unavailable on AnthropicLLM. Keep OpenAI for logprob-based code."
        )
