"""Project-wide baseline vehicle dimensions for the 2D physics model."""

from typing import Final


# The 1.900 m width and 3.400 m wheelbase are the physical constraints used
# for clearance and axle-based surface contact.  The 5.000 m length is the
# current project collision envelope, not an FIA maximum overall length.
PHYSICAL_CAR_WIDTH_M: Final[float] = 1.900
PHYSICAL_CAR_LENGTH_M: Final[float] = 5.000
PHYSICAL_CAR_WHEELBASE_M: Final[float] = 3.400
