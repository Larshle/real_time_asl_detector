
class SentenceAssembler:
    """
    Accumulates letters into words based on gaps in detection,
    debounces repeated letters, and hands off completed words.
    
    - gap_frames: frames with NO letter before ending a word
    - stable_frames: frames the SAME letter must persist before accepting it
    """
    def __init__(self, gap_frames: int = 15, stable_frames: int = 5):
        self.gap_frames       = gap_frames        # no-letter frames → word boundary
        self.stable_frames    = stable_frames     # same-letter frames → letter accepted
        self.letter_history   = []                # collected letters for current word
        self.prev_letter      = None              # last accepted letter
        self.last_seen_letter = None              # last seen frame’s letter
        self.last_seen_count  = 0                 # how many frames we’ve seen it
        self.noletter_cnt     = 0                 # consecutive frames with no letter
        self.words            = []                # completed words

    def update(self, letter: str | None):
        """
        Call once per frame with:
          - A single-letter string (e.g. "A"), or
          - None when no letter detected.

        Returns the completed word (str) if a word boundary is hit, else None.
        """
        # 1) No letter at all → reset seen‐letter, maybe end the word
        if letter is None:
            self.last_seen_letter = None
            self.last_seen_count  = 0
            self.noletter_cnt    += 1
            if self.noletter_cnt >= self.gap_frames and self.letter_history:
                word = "".join(self.letter_history)
                self.words.append(word)
                self.letter_history.clear()
                self.prev_letter   = None
                self.noletter_cnt  = 0
                return word
            return None

        # 2) Saw a letter → reset no-letter counter
        self.noletter_cnt = 0

        # 3) Count consecutive frames of this seen letter
        if letter == self.last_seen_letter:
            self.last_seen_count += 1
        else:
            self.last_seen_letter = letter
            self.last_seen_count  = 1

        # 4) Once stable for enough frames, accept it
        if self.last_seen_count >= self.stable_frames:
            if letter != self.prev_letter:
                self.letter_history.append(letter)
                self.prev_letter = letter

        return None

    def current_sentence(self) -> str:
        """Return all completed words joined as a sentence."""
        return " ".join(self.words)

    def reset(self):
        """Clear all state and start over."""
        self.letter_history.clear()
        self.words.clear()
        self.prev_letter      = None
        self.last_seen_letter = None
        self.last_seen_count  = 0
        self.noletter_cnt     = 0