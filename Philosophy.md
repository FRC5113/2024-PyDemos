# Philosophy of the 5113's Robot Code
> Authors: Vlad Bondar, Ben DeBear
## Simply
1. **It Works**
2. It's Clean
3. It's Efficient
4. It's Concise
5. It's simple

In that order

## Python Style
Style should follow the standard set by [PEP 8](https://peps.python.org/pep-0008/)
Some notes:
* Any style guidelines established by previously written code in this project take precedence over any guideline in PEP 8. That said, any written code that is in violation of PEP 8 and does not follow a preexisting guideline should be amended.
* The process of reformatting code to comply with PEP 8 is greatly facilitated by the tool [Black](https://pypi.org/project/black/), which should be applied to all code before committing.
* Comments that do not contribute to readability are worse than no comments at all
* Previous naming conventions should be abandoned in favor of PEP 8 as those were suited for Java.
* Avoid using prefixes to group names together (eg. "S_Drivetrain" or "P_AutoBalance")
* Annotate function arguments (eg. `arcade_drive(self, forward: float, turn: float)`)

## Magicbot Recommendations
* Use variable injection when possible
* If a component requires a lot of configuration, consider injecting a namedtuple (configuration object should be named cfg)
* Be sure to handle any exceptions that could result from operator input
* Never interact with operator input or networktables in components themselves: this code should only be contained within teleopPeriodic
* Only interact with hardware in the execute method of components

## Github Conventions
* Commit messages should be written in the imperative mood, like a command (eg. "add Philosophy.md")
* Aside from main, branch names should be prefixed by the initials of the main programmer on the branch and an underscore (eg. "kh_pathweaver")
* An exception to this is competition branches, which should be named after the location of the competition (eg. "tabernacle")

_to be completed_