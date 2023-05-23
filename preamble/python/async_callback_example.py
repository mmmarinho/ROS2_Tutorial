from functools import partial
import asyncio
from async_await_example import unlikely_to_return


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


async def main() -> None:
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
    # The functions are already running concurrently. And the result will be processed
    # by the callback. We just wait here until they are over so that the main program
    # does not return prematurely.
    print("Awaiting for results...")
    for (tag, task) in zip(tags, tasks):
        await task


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
