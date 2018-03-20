#!/usr/bin/env python
import curses
import rospy
import Queue
from std_msgs.msg import String

HISTORY_LEN = 500

def main(stdscr):
    # Initialize ROS node
    rospy.init_node("command_prompt", disable_signals=True)

    # Set up split-screen layout using curses
    stdscr.clear()
    in_height = curses.LINES/3
    out_height = curses.LINES - in_height
    in_win = curses.newwin(in_height-1, curses.COLS, 1, 0)
    out_win = curses.newwin(out_height-1, curses.COLS, in_height+1, 0)

    # Draw window headers
    stdscr.attron(curses.A_REVERSE)
    stdscr.hline(0, 0, ' ', curses.COLS)
    stdscr.addstr(0, 0, "OwnageBot Input", curses.A_REVERSE)
    stdscr.hline(in_height, 0, ' ', curses.COLS)
    stdscr.addstr(in_height, 0, "OwnageBot Output", curses.A_REVERSE)
    stdscr.attroff(curses.A_REVERSE)
    stdscr.refresh()

    # Enable scrolling for both windows
    in_win.scrollok(1)
    out_win.scrollok(1)

    # Callback for handling outputs
    def outputCb(msg):
        lines = msg.data.splitlines(True)
        for line in lines:
            # Add newline if missing
            if len(line) > 0 and line[-1] != '\n':
                line += '\n'
            out_win.addstr(line)
        out_win.refresh()
        in_win.refresh() # Switches focus back to input

    # Publish inputs and subscribe to outputs
    input_pub = rospy.Publisher("dialog_in", String, queue_size = 10)
    output_sub = rospy.Subscriber("dialog_out", String, outputCb)

    # Handle inputs until EOL is received
    in_win.keypad(True)
    buf, buf_pos = "", 0
    history, hist_pos = [], 0
    while True:
        try:
            c = in_win.getch()
        except KeyboardInterrupt:
            break

        y, x = in_win.getyx()

        if c in [ord('\n'), ord('\r'), curses.KEY_ENTER]:
            # Publish input upon newline and clear buffer
            if y == in_win.getmaxyx()[0] - 1:
                in_win.scroll()
                in_win.move(y, 0)
            else:
                in_win.move(y+1, 0)
            input_pub.publish(buf)
            history.append(buf)
            if len(history) > HISTORY_LEN:
                history = history[-HISTORY_LEN:]
            hist_pos = 0
            buf, buf_pos = "", 0
        elif c in [ord('\b'), curses.KEY_BACKSPACE]:
            # Delete character upon backspace
            if x > 0:
                in_win.move(y, x-1)
                in_win.delch()
            if buf_pos > 0:
                buf = buf[:buf_pos-1] + buf[buf_pos:]
                buf_pos -= 1
        elif c == curses.KEY_LEFT:
            # Move cursor and buffer position left
            if x > 0:
                in_win.move(y, x-1)
            if buf_pos > 0:
                buf_pos -= 1
        elif c == curses.KEY_RIGHT:
            # Move cursor and buffer position right
            if x < curses.COLS-1 and x < len(buf):
                in_win.move(y, x+1)
            if x < len(buf):
                buf_pos += 1
        elif c == curses.KEY_UP:
            # Go to previous entry in command history
            if hist_pos < len(history):
                hist_pos += 1
                buf = history[-hist_pos]
                buf_pos = len(buf)
                in_win.move(y, 0)
                in_win.clrtoeol()
                in_win.addstr(buf)
                in_win.move(y, len(buf))
        elif c == curses.KEY_DOWN:
            # Go to next entry in command history
            if hist_pos > 0:
                hist_pos -= 1
                buf = "" if hist_pos == 0 else history[-hist_pos]
                buf_pos = len(buf)
                in_win.move(y, 0)
                in_win.clrtoeol()
                in_win.addstr(buf)
                in_win.move(y, len(buf))
        elif c >= 10 and c < 127:
            # Add character
            in_win.insch(c)
            if x < curses.COLS-1:
                in_win.move(y, x+1)
            buf = buf[:buf_pos] + chr(c) + buf[buf_pos:]
            buf_pos += 1
            
        in_win.refresh()

    rospy.signal_shutdown("Received keyboard interrupt.")

if __name__ == '__main__':
    curses.wrapper(main)
