# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'dlib'
copyright = '2024, Cooper Dean, Cody Zhu, Maddy Scott'
author = 'Cooper Dean, Cody Zhu, Maddy Scott'
release = '0.0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "breathe"
]

# Breathe Configuration
breathe_default_project = "dlib"

breathe_projects_source = {
    "dlib" : (
        "../../", ["include/dlib"]
    )
}

breathe_projects = {"dlib": "../xml"}

templates_path = ['_templates']
exclude_patterns = []

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

cpp_maximum_signature_line_length = 100


html_theme = 'furo'
html_static_path = ['_static']