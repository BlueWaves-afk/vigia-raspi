CREATE EXTENSION IF NOT EXISTS postgis;

CREATE TABLE hazard_entities (
    hazard_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    centroid GEOGRAPHY(POINT, 4326) NOT NULL,
    severity_score REAL NOT NULL DEFAULT 0,
    observation_count INTEGER NOT NULL DEFAULT 0,
    first_seen TIMESTAMPTZ NOT NULL,
    last_seen TIMESTAMPTZ NOT NULL,
    status TEXT NOT NULL DEFAULT 'active',
    h3_index TEXT
);

CREATE TABLE device_registry (
    device_id TEXT PRIMARY KEY,
    last_device_seq BIGINT NOT NULL DEFAULT 0,
    hmac_key TEXT NOT NULL,
    created_at TIMESTAMPTZ NOT NULL DEFAULT now()
);

CREATE TABLE observations (
    event_id UUID PRIMARY KEY,
    device_id TEXT NOT NULL,
    device_seq BIGINT NOT NULL,
    hazard_id UUID REFERENCES hazard_entities(hazard_id),
    hazard_class SMALLINT NOT NULL DEFAULT 0,
    observed_at TIMESTAMPTZ NOT NULL,
    location GEOGRAPHY(POINT, 4326),
    rri REAL,
    iss REAL,
    yolo_conf REAL,
    geometry_conf REAL,
    temporal_conf REAL,
    bbox JSONB,
    speed_mps REAL,
    hdop REAL,
    trust_level TEXT NOT NULL,
    raw_payload JSONB,
    ingested_at TIMESTAMPTZ NOT NULL DEFAULT now()
);

CREATE INDEX idx_observations_location ON observations USING GIST (location);
CREATE INDEX idx_observations_device_id ON observations (device_id);
CREATE INDEX idx_observations_observed_at ON observations (observed_at);

CREATE INDEX idx_hazard_entities_centroid ON hazard_entities USING GIST (centroid);
CREATE INDEX idx_hazard_entities_status ON hazard_entities (status);
