#!/bin/bash
# Copies the CPFA functions, unedited, out of Carlos's source files so the test
# runs his real code. Output: carlos_extract.inc (generated, do not edit).
set -e
cd "$(dirname "$0")"
get() {  # file, text that starts the function
    awk -v s="$2" 'index($0,s)==1 && !on && !done {on=1; depth=0; seen=0}
        on {print; line=$0; sub(/\/\/.*/,"",line)
            o=gsub(/{/,"{",line); c=gsub(/}/,"}",line); depth+=o-c; if(o) seen=1
            if(seen && depth==0) {on=0; done=1}}' "$1"
}
C=../../source/CPFA/CPFA_controller.cpp
B=../../source/Base/BaseController.cpp
{
    for f in "void CPFA_controller::CPFA()" "bool CPFA_controller::IsHoldingFood()" \
        "bool CPFA_controller::IsInTheNest()" "void CPFA_controller::Departing()" \
        "void CPFA_controller::Searching()" "void CPFA_controller::Surveying()" \
        "void CPFA_controller::Returning()" "void CPFA_controller::SetRandomSearchLocation()" \
        "void CPFA_controller::SetHoldingFood()" "void CPFA_controller::SetLocalResourceDensity()" \
        "void CPFA_controller::SetFidelityList()" "bool CPFA_controller::SetTargetPheromone()" \
        "argos::Real CPFA_controller::GetExponentialDecay" "argos::Real CPFA_controller::GetBound" \
        "argos::Real CPFA_controller::GetPoissonCDF"; do get "$C" "$f"; done
    for f in "argos::CVector2 BaseController::GetTarget()" "void BaseController::SetTarget(" \
        "void BaseController::SetIsHeadingToNest" "bool BaseController::IsAtTarget()"; do get "$B" "$f"; done
} > carlos_extract.inc
echo "extracted $(grep -cE '^[a-zA-Z].*(CPFA_controller|BaseController)::' carlos_extract.inc) functions"
