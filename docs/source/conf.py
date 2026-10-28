# Configuration file for the Sphinx documentation builder.

# -- Automatic versioning

from datetime import datetime
this_year_str = datetime.today().strftime('%Y')
release_str = datetime.today().strftime('%B %d, %Y')
version_str = datetime.today().strftime('%y.%m.%d')

# -- Project information

project = "(Murilo's) ROS2 Tutorial"
author = 'Murilo M. Marinho'

if this_year_str == '2023':
    copyright = f'2023, {author}'
else:
    copyright = f'2023-{this_year_str}, {author}'

release = release_str
version = version_str

# -- General configuration

extensions = [
    'sphinx.ext.duration',
    'sphinx.ext.doctest',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
    #'hoverxref.extension', # https://sphinx-hoverxref.readthedocs.io/en/latest/index.html
    'sphinx_copybutton', # https://sphinx-copybutton.readthedocs.io/en/latest/
    'sphinx_design', # https://sphinx-design.readthedocs.io/en/latest/get_started.html
    'sphinxext.remoteliteralinclude', # https://github.com/wpilibsuite/sphinxext-remoteliteralinclude
    'sphinxcontrib.mermaid', #https://sphinxcontrib-mermaid-demo.readthedocs.io/en/latest/
    'sphinxcontrib.bibtex', #https://sphinxcontrib-bibtex.readthedocs.io/en/latest/quickstart.html
    'sphinx_reredirects' #https://gitlab.com/jborean93/sphinx-reredirects
]

bibtex_bibfiles = ['refs.bib']

# Old page locations from the topic-directory reorganisation (the pages were
# git mv'ed into per-topic folders, e.g. installation -> setup/installation).
# sphinx_reredirects emits meta-refresh pages for these during the html build.
redirects = {
    'installation': 'setup/installation.html',
    'terminator': 'setup/terminator.html',
    'workspace_setup': 'setup/workspace_setup.html',
    'create_packages': 'packages/create_packages.html',
    'create_python_package': 'packages/create_python_package.html',
    'create_python_node_with_template': 'packages/create_python_node_with_template.html',
    'source_after_build': 'packages/source_after_build.html',
    'running_node': 'packages/running_node.html',
    'create_python_node_from_scratch': 'python_node/create_python_node_from_scratch.html',
    'python_node_explained': 'python_node/python_node_explained.html',
    'create_python_library': 'python_library/create_python_library.html',
    'using_python_library': 'python_library/using_python_library.html',
    'interfaces': 'interfaces/interfaces.html',
    'create_interface_package': 'interfaces/create_interface_package.html',
    'publishers_and_subscribers': 'messages/publishers_and_subscribers.html',
    'inspecting_topics': 'messages/inspecting_topics.html',
    'parameters_and_launch': 'parameters_and_launch/parameters_and_launch.html',
    'launch_configurable_nodes': 'parameters_and_launch/launch_configurable_nodes.html',
    'inspecting_parameters': 'parameters_and_launch/inspecting_parameters.html',
    'service_servers_and_clients': 'services/service_servers_and_clients.html',
    'inspecting_services': 'services/inspecting_services.html',
    'actions': 'actions/actions.html',
    'action_servers': 'actions/action_servers.html',
    'action_clients': 'actions/action_clients.html',
}

intersphinx_mapping = {
    'python': ('https://docs.python.org/3.12/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
}
intersphinx_disabled_domains = ['std']

templates_path = ['_templates']

# -- Options for HTML output

html_theme = 'sphinx_book_theme'
# Tried with `furo` on May 23, 2025, but somehow it didn't look right. In particular the download button wasn't as clear.
html_theme_options = {
    "repository_url": "https://github.com/mmmarinho/ROS2_Tutorial",
    "use_source_button": True,
    "repository_branch": "main",
    "path_to_docs": "docs/source",
    "use_repository_button": True,
    "use_edit_page_button": True,
    "use_issues_button": True,
}
html_title = project

# -- Options for EPUB output
epub_show_urls = 'footnote'

# -- Options for hoverxref.extension https://sphinx-hoverxref.readthedocs.io/en/latest/configuration.html
# hoverxref_auto_ref = True

# https://sphinxcontrib-mermaid-demo.readthedocs.io/en/latest/
mermaid_params = ['-p' 'puppeteer-config.json', '--iconPacks', '@iconify-json/logos', '@iconify-json/material-symbols']
mermaid_output_format = 'png'

# -- Options for latex https://docs.readthedocs.io/en/stable/guides/pdf-non-ascii-languages.html
# latex_engine = "xelatex"
# By using this, the UTF emojis became question marks in the PDF.

# -- Fixing table horizontal bar scrolling https://stackoverflow.com/questions/40641252/how-can-i-avoid-the-horizontal-scrollbar-in-a-rest-table
def setup(app):
        # app.add_stylesheet('custom.css')
        # API Changed https://github.com/sphinx-doc/sphinx/issues/7747
        app.add_css_file('custom.css')


