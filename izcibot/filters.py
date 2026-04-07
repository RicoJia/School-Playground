import subprocess


def filter_min_content_length(item, min_chars):
    """Return True if item's content length is >= min_chars (per-item predicate)."""
    if min_chars <= 0:
        return True
    return len((item.get("content") or "").strip()) >= min_chars


def generate_consolidated_summary(items, model="qwen2.5:7b"):
    """Generate a consolidated summary of multiple related news articles.

    This function takes a list of news items and uses Ollama to generate
    a single coherent summary paragraph that captures the main story across
    all articles, removing redundancy.

    Args:
        items: List of news item dicts with 'title', 'link', 'content' fields
        model: Ollama model to use for summarization

    Returns:
        The same items list, but prints the consolidated summary
    """
    if not items:
        return items

    # Prepare consolidated text from all articles
    articles_text = ""
    for idx, item in enumerate(items, 1):
        articles_text += f"\nArticle {idx}: {item['title']}\n"
        articles_text += f"Content: {item.get('content', '')}\n"

    prompt = f"""You are given multiple news articles about the same story. Generate a single, comprehensive 3-4 sentence summary that:
1. Captures the main facts and developments
2. Eliminates redundancy across articles
3. Presents a cohesive narrative
4. Includes key details like amounts, locations, and parties involved

Articles:
{articles_text}

Provide ONLY the consolidated summary, nothing else."""

    try:
        result = subprocess.run(
            ["ollama", "run", model],
            input=prompt,
            text=True,
            capture_output=True,
            timeout=60,
        )
        summary = result.stdout.strip()

        # Store summary in first item for later use
        if items:
            items[0]["consolidated_summary"] = summary

    except Exception as e:
        print(f"Error generating consolidated summary: {e}")

    return items
