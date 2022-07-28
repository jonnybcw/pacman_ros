import sys, termios;
from select import select;

# save the terminal settings
fd = sys.stdin.fileno();
new_term = termios.tcgetattr(fd);
old_term = termios.tcgetattr(fd);

# new terminal setting unbuffered
new_term[3] = (new_term[3] & ~termios.ICANON & ~termios.ECHO);

def init():
    termios.tcsetattr(fd, termios.TCSAFLUSH, new_term);

def restore():
    termios.tcsetattr(fd, termios.TCSAFLUSH, old_term);

def getch():
    return sys.stdin.read(1);

def kbhit():
    dr,dw,de = select([sys.stdin], [], [], 0);
    return len(dr)>0;
