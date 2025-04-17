# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

from importlib import metadata

copyright = "2025, Austin Gregg-Smith"  # pylint:disable=redefined-builtin
author = "Austin Gregg-Smith"
release = metadata.version("colcon_runner")
project = f"colcon_runner {release}"


# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ["sphinx_rtd_theme", "sphinx.ext.napoleon", "autoapi.extension"]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
# html_static_path = ["_static"]

autoapi_dirs = ["../colcon_runner"]
autoapi_ignore = ["*example_*", "*example*", "*experimental*"]


numpydoc_show_class_members = False

autosummary_generate = True
