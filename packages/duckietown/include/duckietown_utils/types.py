from typing import NewType, TYPE_CHECKING

import numpy as np

__all__ = ['NPImage', 'NPImageBGR', 'NPImageRGB']
if TYPE_CHECKING:
    NPImage = NewType('NPImage', np.ndarray)
    NPImageBGR = NewType('NPImageBGR', np.ndarray)
    NPImageRGB = NewType('NPImageRGB', np.ndarray)

else:

    NPImageBGR = NPImageRGB = NPImage = np.ndarray
