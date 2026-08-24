import csv
import os
import queue
import threading
from dataclasses import asdict


class CSVLogger:

    def __init__(
        self,
        actual_file,
        control_file,
        flush_every=20,
    ):
        self.flush_every = flush_every

        actual_file = os.path.abspath(actual_file)
        control_file = os.path.abspath(control_file)

        os.makedirs(
            os.path.dirname(actual_file),
            exist_ok=True,
        )

        os.makedirs(
            os.path.dirname(control_file),
            exist_ok=True,
        )

        self.actual_fp = open(
            actual_file,
            "w",
            newline="",
            encoding="utf-8",
        )

        self.control_fp = open(
            control_file,
            "w",
            newline="",
            encoding="utf-8",
        )

        self.actual_writer = csv.writer(
            self.actual_fp
        )

        self.control_writer = csv.writer(
            self.control_fp
        )

        self.actual_queue = queue.Queue()
        self.control_queue = queue.Queue()

        self.running = True
        self.counter = 0

        self.thread = threading.Thread(
            target=self._worker,
            daemon=True,
        )

        self.thread.start()

    def write_headers(
        self,
        actual_header,
        control_header,
    ):
        self.actual_writer.writerow(
            actual_header
        )

        self.control_writer.writerow(
            control_header
        )

        self.actual_fp.flush()
        self.control_fp.flush()

    def log_actual(self, actual):
        self.actual_queue.put(
            asdict(actual)
        )

    def log_control(self, control):
        self.control_queue.put(
            asdict(control)
        )

    def _worker(self):
        while (
            self.running
            or not self.actual_queue.empty()
            or not self.control_queue.empty()
        ):
            wrote_data = False

            try:
                actual_data = (
                    self.actual_queue.get(
                        timeout=0.01
                    )
                )

                self.actual_writer.writerow(
                    actual_data.values()
                )

                self.actual_queue.task_done()
                wrote_data = True

            except queue.Empty:
                pass

            try:
                control_data = (
                    self.control_queue.get_nowait()
                )

                self.control_writer.writerow(
                    control_data.values()
                )

                self.control_queue.task_done()
                wrote_data = True

            except queue.Empty:
                pass

            if wrote_data:
                self.counter += 1

            if self.counter >= self.flush_every:
                self.actual_fp.flush()
                self.control_fp.flush()
                self.counter = 0

    def close(self):
        self.running = False

        self.thread.join(
            timeout=2.0
        )

        self.actual_fp.flush()
        self.control_fp.flush()

        self.actual_fp.close()
        self.control_fp.close()