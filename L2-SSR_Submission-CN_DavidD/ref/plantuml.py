"""
An Plantuml extension for generating UML figures from within ipython notebook.
"""
import os
from IPython.core.magic import magics_class, cell_magic, Magics
from IPython.display import Image, SVG

@magics_class
class Plantuml(Magics):

 @cell_magic
 def plantuml(self, line, cell):
    """Generate and display a figure using Plantuml.
    Usage:
        %java -jar plantuml.jar -tsvg filname
    """
    self.filename = line
    self.code = cell

    with open(self.filename + ".plt", "w") as file:
        file.write(self.code)

    os.system("java -jar plantuml.jar -tsvg %s.plt" % self.filename)
    return SVG(filename=self.filename+".svg")

def load_ipython_extension(ipython):
    ipython.register_magics(Plantuml)
