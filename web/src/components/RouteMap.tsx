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

type CoordList = [number, number][];

type OnClickType = (latlng: [number, number]) => void;

// Change view components changes the bounds of the map to fit the route.
const ChangeView: React.FC<{ points: CoordList }> = ({ points }) => {
    const map = useMap();
    if (points.length == 0) {
        // Default to the center of London.
        map.setView([51.505, -0.09], 13);
        return null;
    }
    map.fitBounds(points, { padding: [20, 20] });
    return null;
};

const RouteMap: React.FC<MapProps> = ({ points }) => {
    const markerRef = useRef(null);

    return (
        <MapContainer
            zoom={13}
            scrollWheelZoom={true}
            zoomSnap={0}
            className={styles.mapContainer}
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

export default RouteMap;
