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
import asyncio
import random
from textwrap import dedent


async def unlikely_to_return(tag: str, likelihood: float = 0.1) -> float:
    """
    A function that is unlikely to return.
    :return: When it returns, the successful random roll as a float.
    """
    while True:
        a = random.uniform(0.0, 1.0)
        if a < likelihood:
            print("{} Done.".format(tag))
            return a
        else:
            print(dedent("""\
            {} retry needed (roll = {} > {}).\
            """).format(
                tag,
                a,
                likelihood
            ))
            await asyncio.sleep(0.1)


async def main() -> None:
    tags: list[str] = ["task1", "task2"]
    tasks: list[asyncio.Task] = []

    # Start all tasks before awaiting on them, otherwise the code
    # will not be concurrent.
    for task_tag in tags:
        task = asyncio.create_task(
            unlikely_to_return(tag=task_tag)
        )
        tasks.append(task)

    # Alternatively, use asyncio.gather()
    # The functions are already running concurrently. We are now (a)waiting for the
    # results, one after the other.
    print("Awaiting for results...")
    for (tag, task) in zip(tags, tasks):
        result = await task
        print("The result of task={} was {}.".format(tag, result))


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass