import React, { forwardRef, useEffect, useRef } from "react";
import { Inter } from "next/font/google";
import Head from "next/head";
import styles from "./Map.module.css";
import {
    MapContainer,
    Marker,
    TileLayer,
    Polyline,
    LayerGroup,
    useMap,
} from "react-leaflet";

const icon = L.icon({
    iconUrl: "images/marker-icon.png",
    iconSize: [25, 41],
    iconAnchor: [12.5, 41],
});

// The map component is a React component that renders a map.
// It is used to show the user a map of their route, as well as to allow them to
// interact with the map to choose new route start and end startLocation.

const inter = Inter({ subsets: ["latin"] });

const metadata = {
    title: "JogRoute",
    description: "An automatic route planner for joggers.",
};

interface MapProps {
    startLocation: [number, number]; // The start location of the route.
    setStartLocation: (latlng: [number, number]) => void; // Function to set the start location.
}

// Change view components changes the bounds of the map to the start location.
const ChangeView = ({ startLocation }) => {
    const map = useMap();
    console.log("Changing view to " + startLocation);
    map.setView(startLocation, 13);
    return null;
};

// Clicking on the map will select a new start location for the route.
const MapClick = ({ onClick }) => {
    const map = useMap();
    map.addEventListener("click", (e) => {
        const { lat, lng } = e.latlng;
        onClick([lat, lng]);

        // Move the marker to the new location.
    });
    return null;
};

const LocationSelect: React.FC<MapProps> = ({
    startLocation,
    setStartLocation,
}) => {
    const markerRef = useRef(null);

    // Used to move the marker to the new location when the map is clicked.
    const onClick = (latlng) => {
        setStartLocation(latlng);
        console.log(markerRef.current);
        const marker = markerRef.current;
        console.log("setting marker location to " + latlng);
        marker.setLatLng(latlng);
    };

    return (
        <MapContainer
            center={startLocation}
            zoom={13}
            scrollWheelZoom={true}
            className={styles.mapContainer}
        >
            <TileLayer
                attribution='&copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors'
                url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
            />
            {/*<LayerGroup>
                <Polyline
                    pathOptions={{ color: "purple" }}
                    positions={startLocation}
                />
    </LayerGroup>*/}
            <Marker
                icon={icon}
                position={startLocation}
                draggable={true}
                eventHandlers={{
                    dragend: (e) => {
                        const { lat, lng } = e.target.getLatLng();
                        console.log("Dragged to " + lat + ", " + lng);
                        setStartLocation([lat, lng]);
                    },
                }}
                ref={markerRef}
            />
            {/* <ChangeView startLocation={startLocation} /> */}
            <MapClick onClick={onClick} />
        </MapContainer>
    );
};

export default LocationSelect;
