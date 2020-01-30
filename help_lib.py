
import logging

def create_logger(name):
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)

    formatter = logging.Formatter("[%(asctime)s : %(name)s : %(levelname)s] %(message)s")

    file_handler = logging.FileHandler(f"Logging/{name}.log")
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(formatter)

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(formatter)

    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)

    logger.info("--------Session start--------")

    return logger

def myput(queue, obj):

    try:
        queue.put_nowait(obj)
    except Exception as e:
        pass