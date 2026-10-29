The preregistered process acceptance condition is verified process exit and resource
isolation. Normal application exit and bounded owned-process cleanup are reported
separately. A fully saved, source-valid physical interval can only be accepted
following verified process-group exit and isolated-cache cleanup. A timeout before
source/control completion cannot pass. This clarification was made while D2 was
still initializing, before any new valid control frames/results were available.
