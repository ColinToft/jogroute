import React, { useState } from "react";
import { Inter } from "next/font/google";
import Head from "next/head";
import {
    Button,
    Flex,
    FormControl,
    FormLabel,
    GridItem,
    Heading,
    HStack,
    Link,
    NumberInput,
    NumberInputField,
    SimpleGrid,
    Text,
} from "@chakra-ui/react";

import dynamic from "next/dynamic";
const LocationSelectNoSSR = dynamic(
    () => import("../components/LocationSelect"),
    {
        ssr: false,
    }
);

const inter = Inter({ subsets: ["latin"] });

const metadata = {
    title: "JogRoute",
    description: "An automatic route planner for joggers.",
};

const HomePage: React.FC = () => {
    // Begin with the coordinates of London.
    const [startLocation, setStartLocation] = useState<[number, number]>([
        51.505, -0.09,
    ]);

    const [distance, setDistance] = useState<number>(5);

    const isError = distance < 0 || distance > 20;

    return (
        <div
            className={inter.className}
            style={{
                height: "100vh",
                display: "flex",
                flexDirection: "column",
            }}
        >
            <Head>
                <title>{metadata.title}</title>
                <meta name='description' content={metadata.description} />
            </Head>
            <Heading p='10px' bgGradient='linear(to-r, #20ffff, #4060ff)'>
                JogRoute
            </Heading>
            <SimpleGrid p='10px' columns={2} spacing={10} flex='1'>
                <LocationSelectNoSSR
                    startLocation={startLocation}
                    setStartLocation={setStartLocation}
                />
                <GridItem>
                    <Text>Click on the map to select a start location.</Text>
                    <br />
                    <FormControl isInvalid={isError}>
                        <FormLabel>Distance: </FormLabel>
                        <HStack>
                            <NumberInput
                                defaultValue={distance}
                                min={0}
                                max={20}
                                onChange={(value) => {
                                    setDistance(Number(value));
                                }}
                            >
                                <NumberInputField />
                            </NumberInput>
                            <Text>km</Text>
                        </HStack>
                        <br />

                        <Link
                            href={`/routes?lat=${startLocation[0]}&lon=${
                                startLocation[1]
                            }&distance=${distance * 1000}`}
                        >
                            <Button colorScheme='blue' type='submit'>
                                Generate
                            </Button>
                        </Link>
                    </FormControl>
                </GridItem>
            </SimpleGrid>
        </div>
    );
};

export default HomePage;
