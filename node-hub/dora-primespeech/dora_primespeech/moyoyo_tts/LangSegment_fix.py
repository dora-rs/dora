"""Compatibility layer for LangSegment."""
import re

# Create compatibility functions and classes
def setLangfilters(filters):
    """Compatibility function for setLangfilters."""
    # This is a stub - the actual implementation would filter languages
    pass

class LangSegment:
    """Simplified LangSegment class for compatibility."""
    
    # Class-level filters
    _filters = []
    
    def __init__(self, text=""):
        self.text = text
        self._segments = []
        self._langs = []
        
        # Simple segmentation based on punctuation
        sentences = re.split(r'[。！？!?\n]+', text)
        for sent in sentences:
            if sent.strip():
                self._segments.append(sent.strip())
                # Simple language detection
                if any('\u4e00' <= char <= '\u9fff' for char in sent):
                    self._langs.append("zh")  # Chinese
                else:
                    self._langs.append("en")  # Default to English
    
    @classmethod
    def setfilters(cls, filters):
        """Set language filters."""
        cls._filters = filters
    
    @staticmethod
    def getTexts(text):
        """Static method to get text segments."""
        segments = []
        sentences = re.split(r'[。！？!?\n]+', text)
        for sent in sentences:
            if sent.strip():
                # Simple language detection
                if any('\u4e00' <= char <= '\u9fff' for char in sent):
                    lang = "zh"  # Chinese
                else:
                    lang = "en"  # Default to English
                
                # Return dict format expected by TextPreprocessor
                segments.append({
                    "text": sent.strip(),
                    "lang": lang
                })
        
        if not segments and text.strip():
            # Default to single segment if no splitting occurred
            if any('\u4e00' <= char <= '\u9fff' for char in text):
                lang = "zh"
            else:
                lang = "en"
            segments.append({
                "text": text.strip(),
                "lang": lang
            })
        
        return segments
    
    def getTexts_instance(self):
        """Instance method - Return text segments."""
        return self._segments if self._segments else [self.text]
    
    def getLangs(self):
        """Return language codes."""
        return self._langs if self._langs else ["zh"]

# For compatibility with different import styles
LangSegmentClass = LangSegment