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
from functools import partial
import asyncio
from minimalist_package.minimalist_async import unlikely_to_return


def handle_return_callback(tag: str, future: asyncio.Future) -> None:
    """
    Callback example for asyncio.Future
    :param tag: An example parameter, in this case a tag
    :param future: A asyncio.Future is expected to be the last parameter
    of the callback.
    :return: Nothing.
    """
    if future is not None and future.done():
        print("The result of task={} was {}.".format(tag, future.result()))
    else:
        print("Problem with task={}.".format(tag))


async def async_main() -> None:
    tags: list[str] = ["task1", "task2"]
    tasks: list[asyncio.Task] = []

    # Start all tasks before adding the callback
    for task_tag in tags:
        task = asyncio.create_task(
            unlikely_to_return(tag=task_tag)
        )
        task.add_done_callback(
            partial(handle_return_callback, task_tag)
        )
        tasks.append(task)

    # Alternatively, use asyncio.gather()
    # At this point, the functions are already running concurrently. And the result will be processed
    # by the callback AS "SOON" AS THEY ARE AVAILABLE.
    print("Awaiting for results...")
    for task in tasks:
        await task


def main() -> None:
    try:
        asyncio.run(async_main())
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
