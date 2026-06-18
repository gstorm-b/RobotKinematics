# Agent Handoff Notes

## 2026-06-19 Phase 2 Verification And Phase 3+ Continuation

- Current session scope: verify the previously implemented Phase 2 work, then continue the implementation plan in dependency order toward Phase 8 where feasible.
- Build priority: MSVC first. MinGW compatibility can be checked later unless the current task explicitly requires it.
- Test convention: Qt Test classes live in headers and are registered from `tests/TestMain.cpp`; do not include generated moc files at the end of test source files because that breaks the MSVC build.
- Real robot preset constraint: Kawasaki RS007N and Nachi MZ04D remain blocked until verified dimensions, joint limits, posture rules, and source references are provided.
- Scope discipline: keep core calculations in meters/radians, preserve `Pose` as the canonical transform type, and keep preset data out of solver logic.
