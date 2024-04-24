import streamlit as st
from testing_consolidation_exp import routing as solve_problem


def main():
    st.title("Consolidation Routing Engine")
    
    # File Upload
    st.sidebar.header("Upload CSV File")
    uploaded_file = st.sidebar.file_uploader("Choose a CSV file", type="csv")

    if uploaded_file is not None:
        # Display uploaded file
        st.sidebar.success(f"Uploaded: {uploaded_file.name}")

        solve_problem(uploaded_file)

if __name__ == "__main__":
    main()
