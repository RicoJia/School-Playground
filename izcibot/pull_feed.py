import feedparser
import datetime
import requests
from bs4 import BeautifulSoup


TIME_RANGE = 10


class FetcherBase:
    def __init__(self, name, days=TIME_RANGE):
        self.name = name
        self.days = days
    
    def fetch(self):
        """Abstract method to fetch items. Should return list of dicts with keys: title, link, pub_date, content, source"""
        raise NotImplementedError


def fetch_article_content(url):
    """Fetch and extract main text content from a webpage."""
    try:
        response = requests.get(url, timeout=10)
        response.raise_for_status()
        soup = BeautifulSoup(response.content, 'html.parser')
        
        # Remove script and style elements
        for script in soup(["script", "style"]):
            script.decompose()
        
        # Try to find main content (common patterns)
        article = soup.find('article') or soup.find('main') or soup.find(class_='content')
        if article:
            text = article.get_text(separator=' ', strip=True)
        else:
            text = soup.get_text(separator=' ', strip=True)
        
        # Clean up whitespace
        lines = (line.strip() for line in text.splitlines())
        text = ' '.join(line for line in lines if line)
        
        # Limit to first 3000 words to avoid overwhelming the model
        words = text.split()[:3000]
        return ' '.join(words)
    except Exception as e:
        print(f"Error fetching {url}: {e}")
        return None


class FetchRSS(FetcherBase):
    def __init__(self, name, url, days=TIME_RANGE, fetch_content=True):
        super().__init__(name, days)
        self.url = url
        self.fetch_content = fetch_content

    def fetch(self):
        current_date = datetime.datetime.now()
        cutoff = current_date - datetime.timedelta(days=self.days)
        
        all_items = []
        
        feed = feedparser.parse(self.url)
        recent_items = []
        for entry in feed.entries:
            pub_date = None
            if hasattr(entry, 'published_parsed') and entry.published_parsed:
                pub_date = datetime.datetime(*entry.published_parsed[:6])
            if hasattr(entry, 'updated_parsed') and entry.updated_parsed:
                updated_date = datetime.datetime(*entry.updated_parsed[:6])
                if pub_date is None or updated_date > pub_date:
                    pub_date = updated_date
            if pub_date and pub_date > cutoff:
                content = ""
                if self.fetch_content:
                    content = fetch_article_content(entry.link) or getattr(entry, 'description', '')
                recent_items.append({
                    'title': entry.title,
                    'link': entry.link,
                    'pub_date': pub_date,
                    'content': content,
                    'source': self.name
                })
        
        all_items.extend(recent_items)
        
        # Sort by date
        all_items.sort(key=lambda x: x['pub_date'], reverse=True)
        return all_items


if __name__ == "__main__":
    fetcher = FetchRSS(name="UBC", url="https://www.reddit.com/r/UBC/.rss")
    items = fetcher.fetch()
    print(f"Fetched {len(items)} items from {fetcher.name}")
    for item in items:
        print(f"{item['title']} ({item['link']})")
