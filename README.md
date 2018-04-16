# 📚 SuperimposeMesh Library

A modern C++ augmented-reality library to superimpose 3D objects on images.

[![SuperimposeMesh home](https://img.shields.io/badge/SuperimposeMesh-Home%20%26%20Doc-E0C57F.svg?style=flat-square)](https://robotology.github.io/superimpose-mesh-lib/doxygen/doc/html/index.html) [![Latest Release](https://img.shields.io/github/release/robotology/superimpose-mesh-lib.svg?style=flat-square&label=Latest%20Release)](https://github.com/robotology/superimpose-mesh-lib/releases) [![SemVer](https://img.shields.io/badge/SemVer-2.0.0-brightgreen.svg?style=flat-square)](http://semver.org/) <a href="https://zenhub.com"><img src="https://raw.githubusercontent.com/ZenHubIO/support/master/zenhub-badge.png"></a>



# Overview
- [⚠️ About versioning](#️-about-versioning)
- [📖 Background](#-background)
- [🎛 Dependencies](#-dependencies)
- [🔨 Build and link the library](#-build-and-link-the-library)
- [🔬 Test the library](#-test-the-library)
- [📝 API documentation and example code](#-api-documentaion-and-example-code)
- [📑 Reference](#-reference)


# ⚠️ About versioning
The project is undergoing _heavy_ development: APIs will be subject to changes quite often.
To be able to understand API compatibility during development, the project will follow [SemVer](http://semver.org/) specs.

In particular, the library will have **zero major version**, i.e. **0.MINOR.PATCH**, as specified by [SemVer spec. 4](http://semver.org/#spec-item-4) and the project will comply with the following rules:
1. **MINOR** version increases when API compatibility is broken;
2. **PATCH** version increases when functionality are added in a backwards-compatible manner;
3. Additional labels for pre-release and build metadata are available as extensions to the 0.MINOR.PATCH format.


# 📖 Background
The main interest of the present library is superimposition, which refers to the placement of one thing over another so that both are still evident.
In particular, this library provides function to superimpose 3D mesh model on top of an image, which are of central importance for computer vision and augmented-reality applications.


# 🎛 Dependencies
SuperimposeMesh library depends on
- [GLFW](http://www.glfw.org) - `version >= 3.1`
- [Open Asset Import Library, ASSIMP](http://assimp.org) - `version >= 3.0`
- [OpenGL Extension Wrangler, GLEW](http://glew.sourceforge.net) - `version >= 2.0`
- [OpenCV](http://opencv.org) - `version >= 3.0`
- [OpenGL Mathematics, GLM](http://glm.g-truc.net) - `version >= 0.9`


# 🔨 Build and link the library
Use the following commands to build, install and link the library.

### Build
With `make` facilities:
```bash
$ git clone https://github.com/robotology/superimpose-mesh-lib
$ cd superimpose-mesh-lib
$ mkdir build && cd build
$ cmake ..
$ make
$ [sudo] make install
```

With IDE build tool facilities:
```bash
$ git clone https://github.com/robotology/superimpose-mesh-lib
$ cd superimpose-mesh-lib
$ mkdir build && cd build
$ cmake ..
$ cmake --build . --target ALL_BUILD --config Release
$ cmake --build . --target INSTALL --config Release
```

### Link
Once the library is installed, you can link it using `CMake` with as little effort as writing the following line of code in your poject `CMakeLists.txt`:
```cmake
...
find_package(SuperimposeMesh 0.MINOR.PATCH EXACT REQUIRED)
...
target_link_libraries(<target> SuperimposeMesh::SuperimposeMesh)
...
```


# 🔬 Test the library
We have designed some test to run with `CMake` to see whether everything run smoothly or not. Simply use
```cmake
$ ctest [-VV]
```
to run all the tests.

Tests are also a nice **starting points** to learn how to use the library and how to implement your own filters! _Just have a look at them!_


# 📝 API documentaion and example code
Doxygen-generated documentation is available [here](https://robotology.github.io/bayes-filters-lib/doxygen/doc/html/index.html).


---
[![how-to-export-cpp-library](https://img.shields.io/badge/-Project%20Template-brightgreen.svg?style=flat-square&logo=data%3Aimage%2Fpng%3Bbase64%2CiVBORw0KGgoAAAANSUhEUgAAAEAAAAA9CAYAAAAd1W%2FBAAAABmJLR0QA%2FwD%2FAP%2BgvaeTAAAACXBIWXMAAAsTAAALEwEAmpwYAAAAB3RJTUUH4QEFECsmoylg4QAABRdJREFUaN7tmmuIVVUUx%2F%2F7OmpaaGP6oedkGJWNIWoFVqRZGkIPSrAQgqhEqSYxszeFUB%2FCAqcXUaSRZmZP6IFm42QEUWAjqT1EQ0dLHTMfaWajv76sM%2BxO59znuY%2Bcs2CYmXv33mud31577bX3WU5lEEDOueDvfpLGSBolaaiksyUNknRyqNs%2BSR2SfrKf1ZJaJG11zv1rzJoX4ETgYWAtpcuvwCvABQHcJMUlPevAi5KmxTTbKalN0hZJ2yRlvO%2BOlzTYvOScmP5fSrreOber1mZcQF9gU2j2dgDNwLgixmwE7ge%2BC415FDi%2FFt1%2BuWfkRuBqH1CJYw8B3vfG7wR61NLDn%2BoZt6IcHma%2F7%2FX0zEo6HpRi4KWeYWOTNswfz9OzoKpr3ov2s4HNnmHtwMAy6Vvk6VkPjKkWgInA5zm2r0eBulJn3P6%2FEdgZo2c%2F8BDQu9wP3Qg8DRyIMGJPFhCfAjOAUcAgwOXQ08%2BC3hSb8SMF5AyfANcG4Iteip7L9QMejNjeAlkEjLZ1n490Ah023g%2FAZ0AL8DWwAdgO%2FBnT9y%2Fgdm8CllggbI9ouxeYD4wsNtBcBXwcY8hGYGqo7xjKJyuAyZ6uQ%2Fb5fO%2BzEcCbMf23ANNzeZ6AYcA8oxeWbcDcIAGJWKOlANgCfGNesBR4Cpjqz15ocgIAr0Z4bE%2FgDhsvSt71kzJAAm7O4uJvABfnSmhKBNBY4PL8D4CYdqcBc4CDETp%2Fs3g2SDFGNRVoVCkARhQYlwJ5vgD7JgDLInTvzsT0mQd8BFyTTzrrnGstd84hqR5Y5321LJtNHrABks6V1FfSkVCzeuUxQweAl4Ah2WAAd5XDA4AzgOdCfVbmAe4G22GI2SXATnGFyBrg1rikw05vhcpwIGMBrI%2Bt3UnAMxYgw7Lc7I7Sf7oF0ajcYZ%2BdTBuA24oF4O%2FnS4ErI4w4E3irgLF22f5%2FMEe7r4AJ3vG7y8WBO4Fvs0T%2B8SEb7y4VgC%2B%2FW0QdGFLSC5hmsaRYWWNp7ikRoK%2FL4uLrbZZ7xnhqFwBHske3lZKelfSBc%2B5o6G6wQdJIuxMcIKnBu5FykrZL2iVpq6TVzrm2CMMHS5ouaYak8MPtlfS6pGbn3Ibw3WQYgKTm8LaSpOwHFgCXJHAC7A80AW0xupb4SzGf%2BUx6CeSzxmcBmQLT8Yl2VoiSDZbx9SgSbkUB%2BPKeHZwyMSn1YOBJ4HBM9tYMnFfqNVs1AQTSYQ8zDOgN3AOsi2n7jn%2FxkUTIqgUAuWSTbW3lyi67ANSpdmS3pIWSXnbOra2U0loB8IikJ4JXYJWUTI0AaA%2F260q%2F%2F8uom0sKIAWQAkgBpABSACmAFEAKIAWQAkgBpABSACmAFEB3kc5uBSD0wuUySVN8AB3dgEF%2FK7PdLWmVpOCV3dGMpCGSZkr6%2FliabeA44CagVdIeSXMl1XtNV0kaH%2B58VkQ1RiXklgQBjAYWW11hVLXbfVY2k3OgKfZ%2BvuYB2Bvk2THltIetYOOiYl2pAXgM%2BLkWAHh21dkktcaM2WolgD3DgbCUCDoceK3KAC7MUkO8A5gJ1Fci2DQBP1YCAHCSFWD9EtH3b3Pxy6sVdYdaZVZHEgA8Fw%2Fi0BcxfVqAyUCvklw84STjCuDDEgEMBxbGtPsDeAA4odb34D5WZt%2BeJ4AmK6PZHPHdQeBtYOz%2FNTEZCbwQU%2FaSq0x%2BEtCnqi6eMIxJWUrZAxd%2FPHjoY%2FZQYrnFHIvqh2zNj6uGTf8ARTOPo64fR94AAAAASUVORK5CYII%3D)](https://github.com/robotology/how-to-export-cpp-library)
