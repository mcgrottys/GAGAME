// ================================================================================================
//  PlaceField - a scalar field read at a PLACE, with its absence (the water match, step 3).
//
//  The CPU twin of a field a kernel reads out of a page -- the swell shadow today. Declared here so
//  a hull's water can read it without knowing a GPU exists; compose/ExposurePage answers it with the
//  numbers the page's texels hold. Absence is not zero: Read answers false where the field has no
//  opinion (no field, outside its page), and the READER applies its kernel's absence law, exactly as
//  the kernel does when its own read comes back empty.
// ================================================================================================
#pragma once

namespace ga {

class PlaceField {
public:
    virtual ~PlaceField() = default;
    virtual bool Read(double latDeg, double lonDeg, double& value) const = 0;
};

}  // namespace ga
