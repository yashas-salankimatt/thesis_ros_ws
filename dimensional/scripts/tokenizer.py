#!/usr/bin/python3
import os

class Tokenizer():
    def __init__(self):
        super().__init__('tokenizer')

def main(args=None):
    print("Tokenizer node initialized")
    # get the current directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    print(current_dir)

if __name__ == '__main__':
    main()
