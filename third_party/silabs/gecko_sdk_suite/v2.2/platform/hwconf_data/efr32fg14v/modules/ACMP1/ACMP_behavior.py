from . import ExporterModel
from . import ACMP_model
from . import RuntimeModel


class ACMP(ExporterModel.Module):
    def __init__(self, name=None):
        if not name:
            name = self.__class__.__name__
        super(ACMP, self).__init__(name, visible=True, core=True)
        self.model = ACMP_model

    def set_runtime_hooks(self):
        pass