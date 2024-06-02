"use client"
import { useEffect, useState } from "react";

export default function Home() {
  const [data, setData] = useState(null);

  useEffect(() => {
    async function fetchData() {
      try {
        const resp = await fetch("http://localhost:1337");
        if (!resp.ok) {
          throw new Error("La respuesta de la red no fue correcta");
        }
        const body = await resp.json();
        setData(body);
      } catch (error) {
        console.error(
          "Ha habido un problema con la recuperación de datos:",
          error
        );
        // Fallback data
        setData({
          mode: "normal",
          position: 100,
        });
      }
    }

    fetchData();
  }, []);

  return (
    <div className="min-h-screen flex flex-col items-center justify-center p-4 bg-gradient-to-br from-[#562c9d] to-[#0c001f]">
      <header className="w-full p-4 flex justify-center">
        <img
          src="/logo-ecosun.svg"
          alt="Logo EcoSun Tracker"
          className="w-[403px] h-[166px]"
        />
      </header>
      <main className="flex-1 flex flex-col items-center justify-center text-white p-6">
        <h2 className="text-4xl font-bold mb-4 text-center">
          Maximizando la Recolección de Energía Solar
        </h2>
        <p className="text-lg mb-10 max-w-2xl text-center leading-8">
          El Robot EcoSun Tracker está diseñado para optimizar la recolección de
          energía solar en áreas donde la electricidad es escasa. Ajusta
          automáticamente su orientación para maximizar la captación de energía
          solar, facilitando operaciones autónomas en diversos ambientes y
          asegurando su estabilidad ante condiciones adversas.
        </p>
        {data ? (
          <div className="bg-white p-8 rounded-lg shadow-md text-center w-full max-w-md">
            <p className="text-xl text-[#34008B] font-semibold">
              Modo: {data.mode}
            </p>
            <p className="text-xl text-[#34008B] font-semibold">
              Posición: {data.position}
            </p>
          </div>
        ) : (
          <p className="text-lg">Cargando datos...</p>
        )}
      </main>
      <footer className="w-full p-4 text-center text-white">
        <p>&copy; 2024 EcoSun Tracker. Todos los derechos reservados.</p>
      </footer>
    </div>
  );
}
