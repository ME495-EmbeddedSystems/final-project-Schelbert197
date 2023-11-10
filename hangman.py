from enum import Enum, auto


class State(Enum):
    """Create the states of the node to determine what the timer
    fcn should be doing (MOVING or STOPPED)"""
    MOVING = auto(),
    STOPPED = auto(),
    TELEPORTING = auto()

class Hangman:
    """Plays the game hangman with user input"""

    def __init__(self):
        """Initialize the game vars and other stuff"""

        self.word = "BABIES"
        self.guesses_to_fail = 5
        self.current_wrong_guesses = 0
        self.guessed_letters = []
        self.word_status = ['_','_','_','_','_','_']
        self.game_over = False

    def evaulate_guess(self, guess):
        """Evaluates the guess from the user"""
        
        upper_guess = guess.upper()
        if len(guess) > 1:
            if upper_guess == self.word:
                print('winning')
                for q in range(len(upper_guess)):
                    self.word_status[q] = upper_guess[q]
                self.game_over = True
            else:
                print('word incorrect')
                self.current_wrong_guesses += 1
        elif len(guess) == 1 and upper_guess not in self.guessed_letters:
            if upper_guess in self.word:
                print('letter in word')
                for i in range(len(self.word)):
                    if upper_guess == self.word[i]:
                        self.word_status[i] = upper_guess
                self.check_word()
            else:
                print('wrong guess')
                self.guessed_letters.append(upper_guess)
                self.current_wrong_guesses += 1
        else:
            print('invalid guess')

    def prompt_user(self):
        """User prompt for testing, This would get replaced by OCR pipeline client"""
        user_guess = input("What letter/word is your guess?")
        self.evaulate_guess(user_guess)

    def show_progress(self):
        """Shows the game progress"""
        print(f"Guessed wrong letters: {self.guessed_letters}")
        print(f"Word Status: {self.word_status}")
        print(f"Wrong guesses: {self.current_wrong_guesses}")
        self.draw_man()

    def draw_man(self):
        """lil fella"""
        match self.current_wrong_guesses:

            case 1:
                print("O")
            case 2:
                print("O-")
            case 3:
                print("O-|")
            case 4:
                print("O-|-")
            case 5:
                print("O-|-<  you died")

    def check_word(self):
        """Checks the guessed word"""
        player_word = ''.join(self.word_status)
        if player_word == self.word:
            self.game_over = True
            return True
        else:
            return False

    def play_game(self):
        """plays game for now. Will be replaced with timer callback"""

        while self.current_wrong_guesses < self.guesses_to_fail and self.game_over == False:
            if self.check_word() == False:
                self.prompt_user()
                self.show_progress()

        if self.check_word() == True:
            print("Game won. good job.")
        else:
            print("Game lost, try again later.")
        
test = Hangman()

test.play_game()

