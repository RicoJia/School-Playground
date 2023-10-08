import sys
import select
import tty
import termios

# code pulled from https://www.darkcoding.net/software/non-blocking-console-io-is-not-possible/
# provided by Graham King on various ways to provide keyboard input to python functions
def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def getch():
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        i = 0
        while 1:
            #print(i)
            i += 1

            if isData():
                c = sys.stdin.read(1)
                print(":"+str(c) + ":")
                if c == '\x1b':         # x1b is ESC
                    break
                if c == '\n':
                    break

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
