from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension, build_ext

ext_modules = [
    Pybind11Extension("astar_search_cpp", ["astar_search.cpp"]),
]

setup(
    name="astar_cpp",
    version="1.0",
    author="Michael Hindley",
    description="A* algorithm in C++",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
)
