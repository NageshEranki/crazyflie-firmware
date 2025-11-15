import subprocess
import os

# --- CONFIGURE THIS ONCE ---
# List the paths to *your* code, relative to the
# repository root (the folder with the .git directory).
# This will be the *only* place you need to set this.
MY_CODE_FOLDERS_TO_TRACK = [
    'src/',           # <-- CHANGE THIS: Assumed path to your C code
    'client_code/'  # <-- CHANGE THIS: Path to your python code
]
# ---------------------------

def get_git_hash(repo_path="."):
    """
    Gets the short git hash of a repo at a specific path.
    
    It *only* checks the paths defined in MY_CODE_FOLDERS_TO_TRACK
    for uncommitted changes ("dirty" status).

    Args:
        repo_path (str): The path to the git repository's root 
                         or a folder within it.
    
    Returns:
        str: The 7-character short hash (e.g., "f4a9b2c"), 
             "f4a9b2c-dirty" if there are tracked changes,
             or "unknown" if an error occurs.
    """
    try:
        # Get the 7-character short hash
        git_hash = subprocess.check_output(
            ['git', 'rev-parse', '--short', 'HEAD'], 
            cwd=repo_path, stderr=subprocess.DEVNULL
        ).decode('ascii').strip()
        
        # --- Smarter "Dirty" Check (using the list above) ---
        
        # Base command for checking status
        status_command = ['git', 'status', '--porcelain']
        
        if MY_CODE_FOLDERS_TO_TRACK:
            # Add the hard-coded paths to the command.
            status_command.extend(MY_CODE_FOLDERS_TO_TRACK)
        
        # Run the command for *only* the specified paths
        git_status = subprocess.check_output(
            status_command, 
            cwd=repo_path, stderr=subprocess.DEVNULL
        ).decode('ascii').strip()
        
        if git_status:
            return f"{git_hash}-dirty"
        else:
            return f"{git_hash}"
            
    except subprocess.CalledProcessError:
        return "unknown"
    except FileNotFoundError:
        return "git-not-found"

if __name__ == "__main__":

    hash_str = get_git_hash("../..")
    print(hash_str)