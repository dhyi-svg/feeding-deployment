import json
import time
import uuid
import subprocess
import tempfile
from pathlib import Path
from PIL import Image


class RemoteMolmo:
    def __init__(
        self,
        ssh_host,
        remote_dir="/home/rj277/molmo/sensor_msgs",
        query_timeout_sec=300,
        poll_sec=1.0,
    ):
        self.ssh_host = ssh_host
        self.remote_dir = remote_dir
        self.query_timeout_sec = query_timeout_sec
        self.poll_sec = poll_sec

        self.remote_request_jpg = f"{self.remote_dir}/request.jpg"
        self.remote_request_json = f"{self.remote_dir}/request.json"
        self.remote_response_jpg = f"{self.remote_dir}/response.jpg"
        self.remote_response_json = f"{self.remote_dir}/response.json"

    def _run(self, cmd, check=True, capture_output=False):
        return subprocess.run(
            cmd,
            check=check,
            text=True,
            capture_output=capture_output,
        )

    def _ssh(self, command, check=True, capture_output=False):
        return self._run(
            ["ssh", self.ssh_host, command],
            check=check,
            capture_output=capture_output,
        )

    def _scp_to(self, local_path, remote_path):
        self._run(["scp", str(local_path), f"{self.ssh_host}:{remote_path}"])

    def _scp_from(self, remote_path, local_path):
        self._run(["scp", f"{self.ssh_host}:{remote_path}", str(local_path)])

    def _remote_file_exists(self, remote_path):
        result = self._ssh(f"test -f {remote_path}", check=False)
        return result.returncode == 0

    def _clear_state_files(self):
        self._ssh(
            f"rm -f "
            f"{self.remote_request_jpg} "
            f"{self.remote_request_json} "
            f"{self.remote_response_jpg} "
            f"{self.remote_response_json}"
        )

    def query(self, image_path, prompt, save_response_image_to=None, timeout_sec=None):
        if timeout_sec is None:
            timeout_sec = self.query_timeout_sec

        request_id = str(uuid.uuid4())

        with tempfile.TemporaryDirectory() as tmpdir:
            tmpdir = Path(tmpdir)

            local_request_json = tmpdir / "request.json"
            local_response_json = tmpdir / "response.json"
            local_response_jpg = tmpdir / "response.jpg"

            with open(local_request_json, "w") as f:
                json.dump(
                    {
                        "request_id": request_id,
                        "prompt": prompt,
                    },
                    f,
                )

            self._clear_state_files()

            self._scp_to(image_path, self.remote_request_jpg)
            self._scp_to(local_request_json, self.remote_request_json)

            deadline = time.time() + timeout_sec
            while time.time() < deadline:
                if self._remote_file_exists(self.remote_response_json):
                    self._scp_from(self.remote_response_json, local_response_json)

                    with open(local_response_json, "r") as f:
                        response = json.load(f)

                    if response.get("request_id") != request_id:
                        time.sleep(self.poll_sec)
                        continue

                    if not response.get("ok", False):
                        raise RuntimeError(response.get("error", "Unknown server error"))

                    vis_image = None
                    if self._remote_file_exists(self.remote_response_jpg):
                        self._scp_from(self.remote_response_jpg, local_response_jpg)
                        vis_image = Image.open(local_response_jpg).convert("RGB")

                    pixel_coords = response.get("pixel_coords", [])

                    if save_response_image_to is not None and vis_image is not None:
                        vis_image.save(save_response_image_to)

                    return vis_image, pixel_coords, response

                time.sleep(self.poll_sec)

        raise TimeoutError("Timed out waiting for Molmo response")