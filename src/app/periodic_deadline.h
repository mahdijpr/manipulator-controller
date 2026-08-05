#pragma once

#include <stdint.h>

// Returns the first periodic deadline strictly after nowUs. Keeping the
// previous deadline as the reference prevents processing time from drifting
// the schedule. uint64_t microsecond time will not wrap in practical use.
constexpr uint64_t AdvancePeriodicDeadline(
    uint64_t deadlineUs,
    uint64_t nowUs,
    uint32_t periodUs)
{
    return deadlineUs > nowUs
        ? deadlineUs
        : deadlineUs +
            (static_cast<uint64_t>((nowUs - deadlineUs) / periodUs) + 1U) * periodUs;
}
