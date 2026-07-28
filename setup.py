"""setup.py -- ROS 2 ``ament_python`` packaging shim.

Python packaging/dependency metadata for this repo lives in
``pyproject.toml`` (setuptools ``build_meta`` backend), and stays there --
``pip install -e ".[robot,develop]"`` keeps working exactly as it did before
this migration. This ``setup.py`` exists purely to satisfy what ROS 2's
``ament_python`` build type (and therefore ``colcon build``) expects to
find: an ament resource-index marker so ``ros2 pkg list`` sees this
package, ``package.xml`` installed into ``share/``, and the converted
``launch/``, ``config/``, ``rviz/``, and ``urdf/`` trees installed into the
package's share directory so ``ros2 launch feeding_deployment ...`` and
friends can find them (mirrors what the ROS1 CMakeLists.txt's
``install(DIRECTORY launch config rviz urdf ...)`` did for catkin).

A ``setup.py`` and a PEP 621 ``pyproject.toml`` can coexist fine under the
setuptools backend: ``setup()`` here is called with *only* the data_files
argument, which PEP 621 has no way to express -- name/version/packages/
dependencies/etc. are still sourced entirely from ``pyproject.toml``.

See ``ROS2_MIGRATION_NOTES.md`` for the full ``ament_python`` vs
``ament_cmake`` decision and reasoning.
"""

import os
from glob import glob

from setuptools import setup

package_name = "feeding_deployment"


def _data_files_for(share_subdir: str, source_dir: str, pattern: str = "**/*"):
    """Mirror a source directory tree into ``share/<package>/<share_subdir>``,
    preserving relative subdirectory structure.

    ``data_files`` needs one ``(dest_dir, [files])`` tuple per distinct
    destination directory -- it does not recurse on its own -- so this walks
    ``source_dir`` and buckets files by their destination directory.
    """
    entries = []
    for path in glob(os.path.join(source_dir, pattern), recursive=True):
        if not os.path.isfile(path):
            continue
        rel_dir = os.path.relpath(os.path.dirname(path), source_dir)
        dest = os.path.normpath(os.path.join("share", package_name, share_subdir, rel_dir))
        entries.append((dest, [path]))
    return entries


data_files = [
    (
        "share/ament_index/resource_index/packages",
        [os.path.join("resource", package_name)],
    ),
    (os.path.join("share", package_name), ["package.xml"]),
]
data_files += _data_files_for("launch", "launch")
data_files += _data_files_for("config", "config")
data_files += _data_files_for("rviz", "rviz")
data_files += _data_files_for("urdf", "urdf")

setup(
    data_files=data_files,
)
