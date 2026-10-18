# Pincher underside and default-top references

## Default unrolled top reference

- Original: `IMG_0648_87aeb8.png`
- SHA-256: `5397e7641dee6e197f0d099be780d89c5620bcef5bfa87d32da00eeabbd8e556`
- Size: 2,068,109 bytes; source/archive hashes verified equal.
- Owner identifies this as the default unrolled top pose. The broad smooth upper plate flows longitudinally from the wrist; fingers emerge from opposing side edges, not above the plate. The camera pocket is below that plate. The earlier model's elevated finger plane and proud housing are explicitly rejected by this view. The underside marker plane must not be used as a shell-orientation proxy.
- The paired photos establish qualitative layout, not exact pivot axes, dimensions, finger linkage coupling or pad contacts. Use one shared geometry definition for rendering, collision bodies and planner checks; distinguish photo-estimated dimensions and uncertainty from calibrated optical/marker transforms. Do not present an untested contact estimate as precision metrology.

## Underside reference

Owner-supplied oblique underside photograph, presented sideways by remote positioning of the arm. Screen-up is not robot-up. Retained locally outside Hermes composer/cache cleanup; image remains Git-ignored for home privacy. This is a local archive, not a remote backup.

- Original: `IMG_0647_4ec758.png`
- SHA-256: `83c593205a76be125f210bf333815f6fc997608f525e68ff43022fdf58e9f6cb`
- Size: 2,115,294 bytes; source/archive SHA-256 verified equal.

Owner correction: the pincher is a unified housing with a recessed camera below an adjoining flat region; the two fingers emerge from the other face/edges of that region. It is not separate finger bars plus an external camera blob. The marker is on the underside exterior next to/proximal to the camera recess, not inside the recess. The camera lens belongs inside the cavity; this photograph does not resolve its exact glass diameter or setback.

Known reference: marker68 is the previously established 15 mm black ArUco square, with approximately 20 mm paper from the dense board. Oblique perspective prevents direct screen-pixel ratios from being treated as surveyed 3D housing dimensions. Rounded shell outline, recess, plate, opposed finger-root brackets and bent/rounded fingers are visual evidence. Exact wall thickness, finger joint anatomy, lens setback and top/hidden surfaces remain illustrative.

Update display surfaces only. Preserve optical, marker and contact-surrogate TF, nominal firmware coordinate gauges and planner dimensions unless separately justified by measurements. Verify asset delivery and the actual latched robot description in Foxglove after rebuilding the description package.
