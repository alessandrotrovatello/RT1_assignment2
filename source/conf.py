# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import subprocess
import sys

sys.path.insert(0, os.path.abspath('../'))

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'RT1_Assignment2'
copyright = '2024, Alessandro Trovatello'
author = 'Alessandro Trovatello'
release = '0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
	'sphinx.ext.autodoc',
	'sphinx.ext.doctest',
	'sphinx.ext.intersphinx',
	'sphinx.ext.todo',
	'sphinx.ext.coverage',
	'sphinx.ext.mathjax',
	'sphinx.ext.ifconfig',
	'sphinx.ext.viewcode',
	'sphinx.ext.githubpages',
	"sphinx.ext.napoleon",
	'sphinx.ext.inheritance_diagram'
]

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

source_suffix = '.rst'
master_doc = 'index'
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
