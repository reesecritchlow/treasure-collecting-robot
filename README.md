# __________ Team's Robot Summer GitHub repo

## Branches and Environments

In this repository, there are four-ish main branches:

**Production:** Branch containing the final, _competition_ code. Should be only modified with approval from the team (by means of a pull request), with the understanding that this would be the new code that the robot would run on in the final competition. No edits should be made to this branch directly.

**Staging:** Branch containing the current code running on the robot (most of the time). No edits should be made to this branch directly.

**Dev:** Branch containing new features and more experiemental code, etc. Although not in best practice, edits can be made to this branch if small. If code is truly expriemental in nature, it is best practice that it stays in a personal branch, if not, then the code that should be kept in `dev` should be code that is intented for production in the future. Code in `dev` can be run on the robot, but is not best practice.

**Personal Feature Branches:** Branches which team members work on bug fixes, new features, or experiemental code for the robot. Each branch should be named by the following convention: `initials-type-description`. Types include: `fix` for bug fix, `feature` for a new feature, and `exp` for an experiment. The description should not be super lengthy, but should communicate the intent of the branch's existence. An example of a branch name is: `rc-feature-robot-arm`. Branch names should be in `kebab-case`.

### Workflow

`PFBs` ---(pull request)---> `dev` ---(pull request & version change)---> `staging`---(ceremonial pull request)---> `production`

## Pull Requests

For a PFB, a pull request should be approved by at least one member of the team.

`dev` to `staging` PR's should be approved by at least two members of the team.

`staging` to `prod` PR's should be approved by all members of the team.

To initiate a PR, `git push origin [branch name]` and then create a PR and add reviewers on github.

## Changelog

Changelogs should be updated every time a merge is made from `dev` to `staging`. Version numbers should be reflective of how significant the changes were to the firmware.

Versions should take the form of `v1.2.3` where `1` suggests a massive (pivot) change in design, `2` suggests significant change in design, and `3` suggests a smaller bug fix or feature implementation.

Updating the changelog should entail adding all of the various patches, fixes, and features added in that release.

It is also good practice to utilize `git tag` when merging `dev` to `staging`. For example, a `git tag` command would look like: `git tag -a v1.2.3 -m "Version 1.2.3"`

## Commits

Try to have clean and clear commit messages which are done at appropriate times in the development process. It's hard though.

## ClickUp

If you're feeling awesome add the ClickUp task ID to the PR as CU-taskID (task ID can be found in the task URL), and the PR will be integrated into the ClickUp.

## Code Style

Code should look good. Linting would be nice.
