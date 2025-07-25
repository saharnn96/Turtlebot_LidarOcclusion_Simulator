# Use Python 3.11 slim image as base
FROM python:3.11-slim

# Set working directory
WORKDIR /app

# Set environment variables
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1
ENV DASH_HOST=0.0.0.0
ENV DASH_PORT=8050
ENV DASH_DEBUG=False

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements file
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

# Copy application files
COPY dash_turtlebotsim.py .
COPY Turtlebotsim.py .

# Create a non-root user for security
RUN useradd --create-home --shell /bin/bash turtlebot && \
    chown -R turtlebot:turtlebot /app
USER turtlebot

# Expose the port that Dash runs on
EXPOSE 8050

# Run the Dash application
CMD ["python", "dash_turtlebotsim.py"]
