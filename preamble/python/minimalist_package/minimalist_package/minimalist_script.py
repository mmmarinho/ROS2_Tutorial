#!/bin/python3
import time


def main() -> None:
    """An example main() function that prints 'Howdy!' twice per second."""
    try:
        while True:
            print("Howdy!")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    """When this module is run directly, it's __name__ property will be '__main__'."""
    main()
