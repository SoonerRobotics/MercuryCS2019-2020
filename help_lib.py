
import logging
import logging.handlers
import os

def create_logger(name):
    """Create generic logger for all nodes"""

    # Create logger and let it capture all messages
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)

    # Output formatting
    formatter = logging.Formatter("[ %(asctime)s : %(name)s : %(levelname)s ] %(message)s")

    # Create rotating file handler to backup multiple runs
    filename = f"Logging/{name}"
    if not os.path.exists(filename):
        os.mkdir(filename)
    filename = filename + "/output.log"

    roll_over = os.path.isfile(filename)
    file_handler = logging.handlers.RotatingFileHandler(filename, mode='a', backupCount=5)
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(formatter)
    if roll_over:
        file_handler.doRollover()

    # Create stream handler to print to stdout
    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(formatter)

    # Add handlers to logger
    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)

    logger.info("--------Session start--------")

    return logger

def myput(queue, obj):

    try:
        queue.put_nowait(obj)
    except Exception as e:
        pass