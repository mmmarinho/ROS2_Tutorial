# Configuration file for the Sphinx documentation builder.
from datetime import datetime
this_year_str = datetime.today().strftime('%Y')
today_str = datetime.today().strftime('%y.%m.%d')

# -- Project information

project = 'ROS2 Tutorial'
copyright = '2023, Murilo M. Marinho'
author = 'Murilo M. Marinho'

release = this_year_str
version = today_str

# -- General configuration

extensions = [
    'sphinx.ext.duration',
    'sphinx.ext.doctest',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
    'hoverxref.extension', # https://sphinx-hoverxref.readthedocs.io/en/latest/index.html
    'sphinx_copybutton' # https://sphinx-copybutton.readthedocs.io/en/latest/
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

# -- Options for latex https://docs.readthedocs.io/en/stable/guides/pdf-non-ascii-languages.html
# latex_engine = "xelatex"
# By using this, the UTF emojis became question marks in the pdf.

# -- Fixing table horizontal bar scrolling https://stackoverflow.com/questions/40641252/how-can-i-avoid-the-horizontal-scrollbar-in-a-rest-table
def setup(app):
        # app.add_stylesheet('custom.css')
        # API Changed https://github.com/sphinx-doc/sphinx/issues/7747
        app.add_css_file('custom.css')
