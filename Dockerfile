FROM python:3.10-slim

# Install system dependencies for spatial libraries
RUN apt-get update && apt-get install -y \
    gdal-bin \
    libgdal-dev \
    gcc \
    g++ \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy requirements and install
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY ./app ./app

# Expose port
EXPOSE 8000

# Set Python path
ENV PYTHONPATH=/app

# Command to run
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
