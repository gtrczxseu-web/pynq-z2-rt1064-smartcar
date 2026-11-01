"""Fixed initial reference frames; no odometry integration or hardware access."""
import math

VEHICLE_INITIAL = 'vehicle_initial'
CAMERA_INITIAL = 'camera_initial_y_negative'


def validate_reference(reference):
    if reference not in (VEHICLE_INITIAL, CAMERA_INITIAL):
        raise ValueError('Unknown telemetry reference frame')
    return reference


def wrap_degrees(angle):
    if isinstance(angle, bool) or not math.isfinite(angle):
        raise ValueError('Heading must be finite')
    value=(angle+180.0)%360.0-180.0
    return 0.0 if value==0 else value


def camera_initial_sample(frame, reference):
    """Rotate the fixed initial axes 180deg, retaining the original origin.

    Camera mounting heading is vehicle yaw+180deg in the original axes.
    New axes also rotate 180deg, so camera-relative yaw equals vehicle yaw.
    This is NOT a moving camera/body frame, and never resets on a new capture.
    Speed is a nonnegative magnitude; path length is rotation invariant.
    """
    validate_reference(reference)
    out=dict(frame)
    if reference==VEHICLE_INITIAL:
        return out
    if out.get('reference_frame')==CAMERA_INITIAL:
        return out  # Never flip coordinates twice.
    if out.get('reference_frame') not in (None, VEHICLE_INITIAL):
        raise ValueError('Cannot convert an unknown input reference')
    x,y,heading=(out[k] for k in ('x_grid','y_grid','heading_deg'))
    if any(isinstance(v,bool) or not math.isfinite(v) for v in (x,y,heading)):
        raise ValueError('Coordinates and heading must be finite')
    out.update(reference_frame=CAMERA_INITIAL,vehicle_x_grid=x,vehicle_y_grid=y,
               vehicle_heading_deg=heading,camera_heading_vehicle_deg=wrap_degrees(heading+180),
               x_grid=0.0 if x==0 else -x,y_grid=0.0 if y==0 else -y,
               heading_deg=wrap_degrees(heading))
    for axis in ('x','y'):
        key=axis+'_m'
        if key in out:
            out['vehicle_'+key]=out[key]
            out[key]=0.0 if out[key]==0 else -out[key]
    return out


def reference_description(reference):
    validate_reference(reference)
    return dict(name=reference,
        origin='Stationary calibration: X=Y=path=yaw=0; persists across capture boundaries',
        axes='Calibration camera right=+X, camera forward=+Y' if reference==CAMERA_INITIAL else 'Calibration vehicle right=+X, vehicle forward=+Y',
        camera_mount_yaw_deg=180 if reference==CAMERA_INITIAL else None,
        fixed_axes_rotation_deg=180 if reference==CAMERA_INITIAL else 0,
        heading='Relative yaw from current stationary calibration; anticlockwise positive, clockwise negative',
        pose_computation='ARM midpoint composition of PL body displacement and PL yaw; planar dead reckoning',
        camera_position='Vehicle centre approximation; camera lever arm not calibrated',
        speed='Translation speed magnitude, not signed forward speed',
        distance='Total travelled path length, not signed displacement')
