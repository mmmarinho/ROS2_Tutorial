"""
MIT LICENSE

Copyright (C) 2023 Murilo Marques Marinho (www.murilomarinho.info)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
import time


def main() -> None:
    while True:
        print("Howdy!")
        time.sleep(0.5)


if __name__ == "__main__":
    """
    When this module is run directly, it's __name__ property will be '__main__'.
    
    It is always a good idea to wrap the main() call in a try--except block
    with at least the 'KeyboardInterrupt' clause. This allows the user to shutdown
    the module cleanly. This is of particular importance when hardware is used,
    otherwise the connection with it might be left in an undefined state causing
    difficult-to-understand problems.
    
    The `Exception` clause is very broad, but a must in code that is still under
    development. Exceptions of all sorts can be generated when there is a communication
    error with the hardware, software (internet etc), or other issues.
    This broad clause could be replaced for less broad exception handling if that
    makes sense in a given application, but that is not necessary usually.
    """
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
