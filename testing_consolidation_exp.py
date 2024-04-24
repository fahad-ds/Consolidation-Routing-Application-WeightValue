import numpy as np
import pandas as pd
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import requests
import json
from typing import List
import folium
import osmnx as ox
from folium import plugins
import streamlit as st
import polyline

def routing(uploaded_file):
    def plot_vehicle_route_with_navigation(df, vehicle_name):
        # Filter DataFrame for the specified vehicle
        vehicle_df = df[df['Vehicle'] == vehicle_name]

        # Create a folium map centered at the first stop
        m = folium.Map(location=[vehicle_df.iloc[0]['lat'], vehicle_df.iloc[0]['lng']], zoom_start=12)

        # Iterate over each stop in the vehicle's route
        for index, row in vehicle_df.iterrows():
            # Define popup content
            popup_content = f"""
            <b>Order ID:</b> {row['Order ID']}<br>
            <b>Order Status:</b> {row['Order Status']}<br>
            <b>Order Delivered At:</b> {row['Order Delivered At']}<br>
            <b>Vehicle:</b> {row['Vehicle']}<br>
            <b>Priority:</b> {row['Priority']}<br>
            <b>TFR value (kgs):</b> {row['TFR value (kgs)']}<br>
            <b>Distance (km):</b> {row['Distance (km)']}<br>
            """
            
            # Add marker for each stop with popup
            folium.Marker([row['lat'], row['lng']], popup=popup_content).add_to(m)

        # Generate navigation lines between stops
        for i in range(len(vehicle_df) - 1):
            # Create a list of coordinates for the route segment
            coordinates = [[vehicle_df.iloc[i]['lat'], vehicle_df.iloc[i]['lng']],
                        [vehicle_df.iloc[i + 1]['lat'], vehicle_df.iloc[i + 1]['lng']]]
            
            # Add the route segment to the map
            folium.PolyLine(locations=coordinates, color='blue').add_to(m)

        folium_map_html = m.get_root().render()

        # Display the map using Streamlit
        st.components.v1.html(folium_map_html, width=800, height=600)

    def get_data(file_path: str, max_coordinates: int = 100) -> str:
        """
        Read latitude and longitude data from a CSV file and format it for the API request.

        Args:
            file_path (str): Path to the CSV file containing latitude and longitude data.
            max_coordinates (int, optional): Maximum number of coordinates to consider. Defaults to 100.

        Returns:
            str: Formatted coordinates string for the API request.
        """
        try:
            # Read the CSV file and extract latitude and longitude columns
            df = pd.read_csv(file_path)
            df = df[["lat", "lng"]]

            # Convert DataFrame to a list of tuples and format as "lat,lng"
            coordinates = list(df.itertuples(index=False, name=None))
            coordinates = [f"{c[0]},{c[1]}" for c in coordinates][:max_coordinates]

            # Join the formatted coordinates with a semicolon
            return ";".join(coordinates)
        except FileNotFoundError:
            raise FileNotFoundError("CSV file not found.")
        except Exception as e:
            raise Exception(f"An error occurred while processing the data: {e}")

    def get_matrix(coordinates: str) -> dict:
        """
        Get the distance matrix from the OSRM API for the given coordinates.

        Args:
            coordinates (str): Formatted coordinates string for the API request.

        Returns:
            dict: JSON response containing the distance matrix and annotations.
        """
        try:
            url = f"http://router.project-osrm.org/table/v1/driving/{coordinates}?annotations=distance"
            response = requests.get(url)

            # Raise an exception if the API request was not successful
            response.raise_for_status()

            # Parse the JSON response and return the result as a dictionary
            return json.loads(response.text)
        except requests.exceptions.RequestException as e:
            raise requests.exceptions.RequestException(f"API request failed: {e}")
        except json.JSONDecodeError:
            raise ValueError("Invalid JSON response from the API.")
        except Exception as e:
            raise Exception(f"An error occurred while fetching the distance matrix: {e}")

    def parse_coordinates(df: pd.DataFrame) -> str:
        """
        Parse latitude and longitude data from a DataFrame and format it for the API request.

        Args:
            df (pd.DataFrame): DataFrame containing 'lat' and 'lng' columns.

        Returns:
            str: Formatted coordinates string for the API request.
        """
        # Extract latitude and longitude columns from the DataFrame
        coordinates = df[["lat", "lng"]]

        # Convert DataFrame to a list of tuples and format as "lng,lat"
        coordinates = list(coordinates.itertuples(index=False, name=None))
        coordinates = [f"{c[1]},{c[0]}" for c in coordinates]

        # Join the formatted coordinates with a semicolon
        return ";".join(coordinates)

    def create_data_model(distances, coordinates):
        """Stores the data for the problem."""
        data = {}
        data["distance_matrix"] = distances
        data["demands"] = list(map(int, places['Total Weight'].to_list())) 
        data["vehicle_capacities"] = [1000] * Ravi_Count + [2500] * Shehzore_Count + [7000] * Mazda_Count + [600] * EV_Count
        data["num_vehicles"] = Ravi_Count + Shehzore_Count + Mazda_Count + EV_Count
        counts = {'Ravi': Ravi_Count, 'Shehzore': Shehzore_Count, 'Mazda': Mazda_Count, 'EV': EV_Count}
        # Generate a list of tuples containing the vehicle type and its count
        vehicles_with_counts = [(key, value) for key, value in counts.items()]
        # Generate the vehicle list using list comprehension and arithmetic operations
        data['vehicle_list'] = [f'{key}_{i}' for key, value in vehicles_with_counts for i in range(value, 0, -1)]

        data["depot"] = 0
        coordinates = [tuple(float(x) for x in coord.split(",")) for coord in coordinates.split(";")]
        data["coordinates"] = coordinates
        return data

    def build_solution_dataframe1(data, manager, routing, solution):
        # Initialize lists to store parsed information
        vehicles = []
        priorities = []
        sequences = []
        distances = []
        loads = []

        # Iterate over vehicles
        for vehicle_id in range(data["num_vehicles"]):
            index = routing.Start(vehicle_id)
            route_distance = 0
            route_load = 0
            route = []

            # Build the route for the vehicle
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                route_load += data["demands"][node_index]
                route.append(node_index)
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

            route.append(manager.IndexToNode(index))

            # Append information to lists
            vehicles.append(data['vehicle_list'][vehicle_id])
            distances.append(route_distance)
            loads.append(route_load)
            
            # Extracting priority and sequence information
            priorities.append([priority for priority, seq in enumerate(route)])
            sequences.append(route)

        # Create DataFrame
        df = pd.DataFrame({
            'Vehicle': vehicles,
            'Priorities': priorities,
            'Sequence': sequences,
            'Distance (m)': distances,
            'Load': loads
        })
        return df
    
    def build_solution_dataframe11(data):

        # Initialize lists to store parsed information
        vehicles = []
        routes = []
        distances = []
        loads = []

        # Iterate over rows in the dataframe
        for index, row in data.iterrows():
            vehicle = row['Vehicle']
            route = row['Route']
            distance = row['Distance (m)']
            load = row['Load']

            # Split the route string to extract sequence and load for each stop
            stops = route.strip().split('->')

            # Iterate over stops to extract sequence and load
            sequences = []
            stop_loads = []
            for stop in stops:
                stop = stop.strip()
                if stop:
                    sequence, load_str = stop.split('Load(')
                    sequence = int(sequence.strip())
                    load = int(load_str[:-1].strip())  # Remove the closing parenthesis
                    sequences.append(sequence)
                    stop_loads.append(load)

            # Append information to lists
            vehicles.extend([vehicle] * len(sequences))
            routes.extend(sequences)
            distances.extend([distance] * len(sequences))
            loads.extend(stop_loads)

        # Create DataFrame
        df = pd.DataFrame({
            'Vehicle': vehicles,
            'Sequence': routes,
            'Distance (m)': distances,
            'Load': loads
        })
        return df

    def build_solution_dataframe111(dff, new_df, data):
        # Initialize list to store parsed information
        distances = []

        # Iterate over rows in the dataframe
        for index, row in dff.iterrows():
            # Split the route string to extract sequence for each stop
            stops = row['Route'].strip().split('->')

            # Initialize list to store distances for the current route
            route_distances = []
            for i in range(len(stops) - 1):
                # Extract sequence for current and next stop
                current_sequence = int(stops[i].split()[0])
                next_sequence = int(stops[i + 1].split()[0])

                # Extract distance between current and next stop from the distance matrix
                distance = data["distance_matrix"][current_sequence][next_sequence]
                route_distances.append(distance)
            route_distances = [0] + route_distances
            # Append distances for the current route to the distances list
            distances.extend(route_distances)

        #distances.insert(0, 0)
        # Add distances as a new column to the dataframe
        new_df['Distance (m)'] = distances

        return new_df

    def build_solution_dataframe(data, manager, routing, solution):
        # Initialize lists to store parsed information
        vehicles = []
        routes = []
        distances = []
        loads = []

        # Iterate over vehicles
        for vehicle_id in range(data["num_vehicles"]):
            index = routing.Start(vehicle_id)
            route_distance = 0
            route_load = 0
            route = ""

            # Build the route for the vehicle
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                route_load += data["demands"][node_index]
                route += f" {node_index} Load({route_load}) -> "
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

            route += f" {manager.IndexToNode(index)} Load({route_load})"

            # Append information to lists
            vehicles.append(data['vehicle_list'][vehicle_id])
            routes.append(route.strip())
            distances.append(route_distance)
            loads.append(route_load)

        # Create DataFrame
        df = pd.DataFrame({
            'Vehicle': vehicles,
            'Route': routes,
            'Distance (m)': distances,
            'Load': loads
        })
        return df

    def main(places):

        coordinates = parse_coordinates(places)
        response_distances = get_matrix(coordinates)
        distances = np.array(response_distances["distances"]).astype(int)

        """Solve the CVRP problem."""
        # Instantiate the data problem.
        data = create_data_model(distances, coordinates)

        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(
            len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
        )

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)

        # Create and register a transit callback.
        def distance_callback(from_index, to_index):
            """Returns the distance between the two nodes."""
            # Convert from routing variable Index to distance matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return data["distance_matrix"][from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)

        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Add Capacity constraint.
        def demand_callback(from_index):
            """Returns the demand of the node."""
            # Convert from routing variable Index to demands NodeIndex.
            from_node = manager.IndexToNode(from_index)
            return data["demands"][from_node]

        demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
        routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,
            0,  # null capacity slack
            data["vehicle_capacities"],  # vehicle maximum capacities
            True,  # start cumul to zero
            "Capacity",
        )

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        search_parameters.time_limit.FromSeconds(1)

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        if solution:
            dff = build_solution_dataframe(data, manager, routing, solution)
            dff = dff[dff['Load'] > 0].reset_index(drop=True)
            #st.dataframe(dff)

            dff1 = build_solution_dataframe1(data, manager, routing, solution)
            dff1 = dff1[dff1['Load'] > 0].reset_index(drop=True)
            dff1 = dff1.rename(columns={'Load': 'TFR value (kgs)'})
            dff1['Distance (km)'] = dff1['Distance (m)'] / 1000
            dff1.drop(columns=['Distance (m)'], inplace=True)
            st.subheader("Route Level Results")
            st.dataframe(dff1)
            
            new_df = build_solution_dataframe11(dff)

            def map_sequence_to_order_id(sequence):
                return places.iloc[sequence]['Order ID']
            
            def map_sequence_to_lat(sequence):
                return places.iloc[sequence]['lat']
            
            def map_sequence_to_lng(sequence):
                return places.iloc[sequence]['lng']
            
            def map_sequence_to_OrderDA(sequence):
                return places.iloc[sequence]['Order Delivered At']
            
            def map_sequence_to_OrderS(sequence):
                return places.iloc[sequence]['Order Status']

            # Map sequence to order ID and create a new column 'Order ID'
            new_df['lat'] = new_df['Sequence'].apply(map_sequence_to_lat)
            new_df['lng'] = new_df['Sequence'].apply(map_sequence_to_lng)

            new_df['Order Delivered At'] = new_df['Sequence'].apply(map_sequence_to_OrderDA)
            new_df['Order Status'] = new_df['Sequence'].apply(map_sequence_to_OrderS)
            
            new_df['Order ID'] = new_df['Sequence'].apply(map_sequence_to_order_id)
            
            # Rename 'Load' column to 'TFR value (kgs)' and calculate TFR value per order for each vehicle
            new_df = new_df.rename(columns={'Load': 'TFR value (kgs)'})
            new_df['TFR value (kgs)'] = new_df.groupby('Vehicle')['TFR value (kgs)'].diff().fillna(new_df['TFR value (kgs)'])

            new_df = build_solution_dataframe111(dff, new_df, data)

            new_df['Distance (km)'] = new_df['Distance (m)'] / 1000
            new_df.drop(columns=['Distance (m)'], inplace=True)
            new_df.drop(columns=['Sequence'], inplace=True)
            new_df['Priority'] = new_df.groupby('Vehicle').cumcount() + 1
            new_df = new_df[['Order ID', 'Order Status', 'Order Delivered At', 'Vehicle', 'Priority', 'TFR value (kgs)', 'Distance (km)', 'lat', 'lng']]


            st.subheader("Order Level Results")
            st.dataframe(new_df)


            for i in new_df.Vehicle.unique():
                vehicle = "Visualization: " + str(i)
                st.subheader(vehicle)
                plot_vehicle_route_with_navigation(new_df, i)

        else:
            st.write("No Solution Found with given vehicles.")

    places1 = pd.read_csv(uploaded_file)

    Ravi_Count = 100 #st.number_input("Input number of RV available: ", min_value=0, max_value=100, value=50)#100 #int(input("Input number of Ravi available: "))
    Shehzore_Count = 100 #st.number_input("Input number of Shehzore available: ", min_value=0, max_value=100, value=50)#100 #int(input("Input number of Shehzore available: "))
    Mazda_Count = 100 #st.number_input("Input number of Mazda available: ", min_value=0, max_value=100, value=50)#100 #int(input("Input number of Mazda available: "))
    EV_Count = 100 #st.number_input("Input number of EV available: ", min_value=0, max_value=100, value=50)#100 #int(input("Input number of EV available: "))

    new_df = pd.DataFrame()

    submit_button = st.button("Process")

    if submit_button:
        for i in places1['Seller Latitude'].unique():
            places = places1[places1['Seller Latitude'] == i]
            places = places.fillna(0)
            places['Total Weight'] = places['Total Weight'].apply(lambda x: 1 if x < 1 else x)
            places = places.reset_index(drop=True)
            st.write("Consolidation Centre - Latitude: ", places['Seller Latitude'][0], "Longitude: ", places['Seller Longitude'][0])
            st.dataframe(places)
            # Create a new row as a pandas Series
            new_row = pd.Series({'Order ID': 0, 'Order Delivered At': 0, 'Order Status': 0, 'lat': i, 'lng': places['Seller Longitude'].to_list()[0], 'Total Weight': 0})

            # Concatenate the new row with the existing DataFrame
            places = pd.concat([new_row.to_frame().T, places], ignore_index=True)

            # Reset the index
            places.reset_index(drop=True, inplace=True)
            main(places)