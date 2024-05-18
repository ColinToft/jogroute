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
import L from "leaflet";

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
    points: [number, number][]; // The points on the route
}

// Change view components changes the bounds of the map to fit the route.
const ChangeView: React.FC<MapProps> = ({ points }) => {
    const map = useMap();
    map.fitBounds(points, { padding: [5, 5] });
    return null;
};

const LocationSelect: React.FC<MapProps> = ({ points }) => {
    const markerRef = useRef(null);

    return (
        <MapContainer
            zoomControl={false}
            attributionControl={false}
            scrollWheelZoom={false}
            doubleClickZoom={false}
            touchZoom={false}
            closePopupOnClick={false}
            dragging={false}
            trackResize={false}
            zoomSnap={0}
            className={styles.mapContainerSmall}
        >
            <TileLayer
                attribution='&copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors'
                url='https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png'
            />
            <LayerGroup>
                <Polyline pathOptions={{ color: "blue" }} positions={points} />
            </LayerGroup>
            {/*<Marker
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
            />*/}
            <ChangeView points={points} />
        </MapContainer>
    );
};

export default LocationSelect;
