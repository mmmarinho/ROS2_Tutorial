# Configuration file for the Sphinx documentation builder.

# -- Project information

project = 'ROS2 Tutorial'
copyright = '2023, Murilo M. Marinho'
author = 'Murilo M. Marinho'

release = '0.1'
version = '0.1.0'

# -- General configuration

extensions = [
    'sphinx.ext.duration',
    'sphinx.ext.doctest',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
    'hoverxref.extension' # https://sphinx-hoverxref.readthedocs.io/en/latest/index.html
]

intersphinx_mapping = {
    'python': ('https://docs.python.org/3.10/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
}
intersphinx_disabled_domains = ['std']

templates_path = ['_templates']

# -- Options for HTML output

html_theme = 'sphinx_rtd_theme'

# -- Options for EPUB output
epub_show_urls = 'footnote'

# -- Options for hoverxref.extension https://sphinx-hoverxref.readthedocs.io/en/latest/configuration.html
hoverxref_auto_ref = True

# Optionally declare the Python requirements required to build your docs
python:
    install:
        - requirements: docs/source/requirements.txt
