from geopy.distance import geodesic
import time
from config import (
    MAX_COLLECTIONS,
    WP_RADIUS_LIMIT,
    BOAT_RADIUS_LIMIT,
    MIN_TRASH_DUPLICATE_DISTANCE
)

class DecisionEngine:
    def __init__(self, gps_required=True):
        self.max_collections = MAX_COLLECTIONS
        self.wp_radius_limit = WP_RADIUS_LIMIT
        self.boat_radius_limit = BOAT_RADIUS_LIMIT
        self.min_distance_between_trash = MIN_TRASH_DUPLICATE_DISTANCE
        self.gps_required = gps_required
        self.collected_trash_coords = []

    def haversine(self, lat1, lon1, lat2, lon2):
        return geodesic((lat1, lon1), (lat2, lon2)).meters

    def is_duplicate(self, lat, lon):
        for prev_lat, prev_lon, _ in self.collected_trash_coords:
            if self.haversine(lat, lon, prev_lat, prev_lon) <= self.min_distance_between_trash:
                return True
        return False

    def evaluate_detection(self, trash_lat, trash_lon, boat_lat, boat_lon, wp_lat, wp_lon):
        if self.gps_required and any(coord is None for coord in [trash_lat, trash_lon, boat_lat, boat_lon]):
            return {"action": "ignore", "reason": "no_gps"}
        if len(self.collected_trash_coords) >= self.max_collections:
            return {"action": "ignore", "reason": "max_limit"}
        if self.is_duplicate(trash_lat, trash_lon):
            return {"action": "ignore", "reason": "duplicate"}

        dist_to_wp = self.haversine(trash_lat, trash_lon, wp_lat, wp_lon)
        dist_to_boat = self.haversine(trash_lat, trash_lon, boat_lat, boat_lon)

        print(f"[DECISION] Trash→WP: {dist_to_wp:.2f}m, Trash→Boat: {dist_to_boat:.2f}m")

        if dist_to_wp <= self.wp_radius_limit and dist_to_boat <= self.boat_radius_limit:
            return {"action": "collect", "reason": "within_range", "lat": trash_lat, "lon": trash_lon}
        elif dist_to_wp <= self.wp_radius_limit:
            return {"action": "wait", "reason": "boat_not_close"}

        return {"action": "ignore", "reason": "too_far"}

    def mark_collected(self, lat, lon):
        self.collected_trash_coords.append((lat, lon, time.time()))
        print(f"[COLLECTED] Trash marked at lat={lat:.6f}, lon={lon:.6f}")
