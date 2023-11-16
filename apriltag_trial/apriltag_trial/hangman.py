from enum import Enum, auto
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from matplotlib.textpath import TextToPath
from matplotlib.font_manager import FontProperties


class State(Enum):
    """Create the states of the node to determine what the timer
    fcn should be doing (MOVING or STOPPED)"""
    PLAYING = auto(),
    GAME_OVER = auto(),
    WAITING = auto()

class Hangman(Node):
    """Plays the game hangman with user input"""

    def __init__(self):
        super().__init__("hangman")
        """Initialize the game vars and other stuff"""

        self.state = State.WAITING
        self.word = "BABIES"
        self.guesses_to_fail = 5
        self.current_wrong_guesses = 0
        self.guessed_letters = []
        self.word_status = ['_','_','_','_','_','_']
        self.game_won = False
        self.user_guess = None
        self.Alphabet = {}

        # Create Timer
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Create Subscribers
        self.input = self.create_subscription(String, "/user_input", self.user_input_callback, qos_profile=10)

        self.create_letters()

    def create_letters(self):
        """Create the dictionary of bubble letters"""

        letters = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'
        for i in range(0,len(letters)):
            letter = letters[i]
            fp = FontProperties(family="MS Gothic", style="normal")
            verts, codes = TextToPath().get_text_path(fp, letters[i])
            # print(type(path))
            # print(verts)
            xlist = []
            ylist = []
            for j in range(0,len(verts)-1):
                if verts[j][0] > 0:
                    xlist.append(verts[j][0])
                    ylist.append(verts[j][1])
            point_dict = {letter: {'xlist': xlist,'ylist':ylist}}
            self.Alphabet.update(point_dict)

    def evaulate_guess(self, guess):
        """Evaluates the guess from the user"""
        
        upper_guess = guess.upper()
        if len(guess) > 1:
            if upper_guess == self.word:
                self.get_logger().info('winning')
                for q in range(len(upper_guess)):
                    self.word_status[q] = upper_guess[q]
                self.game_won = True
            else:
                self.get_logger().info('word incorrect')
                self.current_wrong_guesses += 1
        elif len(guess) == 1 and upper_guess not in self.guessed_letters:
            if upper_guess in self.word:
                self.get_logger().info('letter in word')
                for i in range(len(self.word)):
                    if upper_guess == self.word[i]:
                        self.word_status[i] = upper_guess
                # self.check_word()
            else:
                self.get_logger().info('wrong guess')
                self.guessed_letters.append(upper_guess)
                self.current_wrong_guesses += 1
        else:
            self.get_logger().info('invalid guess')

    def prompt_user(self):
        """User prompt for testing, This would get replaced by OCR pipeline client"""
        user_guess = input("What letter/word is your guess?")
        self.evaulate_guess(user_guess)

    def show_progress(self):
        """Shows the game progress"""
        self.get_logger().info(f"Guessed wrong letters: {self.guessed_letters}")
        self.get_logger().info(f"Word Status: {self.word_status}")
        self.get_logger().info(f"Wrong guesses: {self.current_wrong_guesses}")
        self.draw_man()

    def draw_man(self):
        """lil fella"""
        match self.current_wrong_guesses:

            case 1:
                self.get_logger().info("O")
            case 2:
                self.get_logger().info("O-")
            case 3:
                self.get_logger().info("O-|")
            case 4:
                self.get_logger().info("O-|-")
            case 5:
                self.get_logger().info("O-|-<  you died")

    def check_word(self):
        """Checks the guessed word"""
        player_word = ''.join(self.word_status)
        if player_word == self.word:
            self.game_won = True
            self.state == State.GAME_OVER
            return True
        else:
            return False

    def play_game(self):
        """plays game for now. Will be replaced with timer callback"""

        while self.current_wrong_guesses < self.guesses_to_fail and self.game_won == False:
            if self.check_word() == False:
                self.prompt_user()
                self.show_progress()

        if self.check_word() == True:
            self.get_logger().info("Game won. good job.")
        else:
            self.get_logger().info("Game lost, try again later.")

    def user_input_callback(self, msg:String):
        """Callback for the user input subscriber (check prompt user)"""
        self.user_guess = msg.data
        self.get_logger().info(f"Message: {self.user_guess}")
        # self.evaulate_guess(self.user_guess)

    def timer_callback(self):
        """Timer callback for the game to play hangman (check play game)"""
        if self.state == State.PLAYING:
            self.evaulate_guess(self.user_guess)
            if self.current_wrong_guesses < self.guesses_to_fail:
                if self.check_word() == False:
                    # self.prompt_user()
                    self.show_progress()
                    self.state = State.WAITING

                if self.check_word() == True:
                    self.get_logger().info("Game won. good job.")
                    self.show_progress()
                    self.state = State.GAME_OVER
            else:
                self.get_logger().info("Game lost, try again later.")
                self.show_progress()
                self.state = State.GAME_OVER

        if self.state == State.WAITING:
            # Check if the player has done something
            if self.user_guess == None:
                pass
            elif self.user_guess in self.guessed_letters or self.user_guess in self.word_status:
                pass
            else:
                self.state = State.PLAYING

        if self.state == State.GAME_OVER:
            pass
        
# test = Hangman()

# test.play_game()

def main(args=None):
    """ The node's entry point """
    rclpy.init(args=args)
    node = Hangman()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()